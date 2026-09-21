#include "espnow.h"

#include <string.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace touge {

FastRadio fastRadio;

namespace {

// Eight frames is a little under a second of speech at three 20 ms codec
// frames per packet. Deeper would only add delay to audio that is already
// late; shallower drops packets when the module misses a scheduling slot.
// Frames the WiFi callback can hand over before it starts dropping.
//
// Eight was a bench number. A burst from twenty-eight cars fills it in one
// cycle and dropped climbs while every phone still reports a healthy lane,
// because a frame refused at the queue was never heard as far as anything
// above it is concerned.
// Eight, not thirty-two. Each FastRx carries a full frame, so the queue is
// RX_DEPTH * ~260 bytes of heap, allocated up front by xQueueCreate. Thirty-two
// was 8 KB, and on a no-PSRAM V3 that 8 KB is exactly what NimBLE needed and
// could not get - an unguarded `new` in BLE advertising setup then aborted the
// device into a boot loop. Eight is ~2 KB and still a quarter-second of frames
// at the fast-lane rate, which is all the drain loop can fall behind by.
const int RX_DEPTH = 8;

QueueHandle_t rxQueue = nullptr;
volatile uint32_t dropCount = 0;
volatile uint32_t sendFailCount = 0;
volatile int lastSendErr = 0;

// Read by the receive callback, which has no handle on the FastRadio.
volatile uint8_t currentChannel = 0;

// Whether the radio accepted the transmit power we asked for.
bool txPowerSet = false;

const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// arduino-esp32 3.x (ESP-IDF 5.x) hands the callback a struct carrying the
// receive control block, which is where the RSSI lives. Older cores pass a
// bare MAC and there is no RSSI to be had. Both are supported because a
// Meshtastic checkout may be pinned to either.
#if ESP_IDF_VERSION_MAJOR >= 5
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  int8_t rssi = (info && info->rx_ctrl) ? (int8_t)info->rx_ctrl->rssi : 0;
#else
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
  int8_t rssi = 0;
#endif
  if (rxQueue == nullptr || len <= 0 || len > (int)FRAME_MAX) return;

  FastRx rx;
  rx.len = (uint16_t)len;
  rx.rssi = rssi;
  rx.rxMs = millis();
  rx.chan = currentChannel;
  memcpy(rx.data, data, (size_t)len);

  // This runs on the Wi-Fi task. Blocking here stalls the driver, so a full
  // queue drops the frame and says so rather than waiting for room.
  if (xQueueSend(rxQueue, &rx, 0) != pdTRUE) dropCount++;
}

bool startPeer() {
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = 0; // whatever channel the interface is already on
  peer.ifidx = WIFI_IF_STA;
  // ESP-NOW's own encryption is not used. It is limited to six encrypted
  // peers, needs pairwise keys, and does not work with broadcast at all. The
  // payload is already encrypted under the ride's key before it gets here.
  peer.encrypt = false;

  esp_err_t err = esp_now_add_peer(&peer);
  return err == ESP_OK || err == ESP_ERR_ESPNOW_EXIST;
}

} // namespace

bool FastRadio::begin(const FastNet& net) {
  if (!net.valid) return false;
  if (ready_) return retune(net);

  if (rxQueue == nullptr) {
    rxQueue = xQueueCreate(RX_DEPTH, sizeof(FastRx));
    if (rxQueue == nullptr) return false;
  }

  // Station mode with no connection. Meshtastic may also want Wi-Fi; if it has
  // already joined an access point, the AP owns the channel and the call below
  // will not move us off it. That is correct behaviour, not a failure: the
  // ride simply runs on the AP's channel instead of the derived one, and the
  // boards still agree because they all read the same interface.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);

#if TOUGE_FAST_LONG_RANGE
  // Has to be set before esp_now_init, and on the interface ESP-NOW will use.
  // A node with LR on cannot hear a node with LR off, so this is all or none
  // across the ride.
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N |
                                         WIFI_PROTOCOL_LR);
#endif

  // Ask for the full legal transmit power rather than whatever was left set.
  //
  // Long-range mode buys sensitivity at the receiver; this is the other half,
  // and it was simply never asked for. The driver's default is not guaranteed
  // to be the maximum and anything else on the system that has touched Wi-Fi
  // may have lowered it, so a board could be running a long-range link at
  // reduced power and nothing anywhere would say so.
  //
  // Units are quarter-dBm, so 80 is 20 dBm - the ceiling the ESP32 will accept
  // and inside 2.4 GHz limits in the regions this is used. A failure here is
  // not fatal: a quieter radio still works, it just does not reach as far.
  if (esp_wifi_set_max_tx_power(80) != ESP_OK) {
    // Nothing to do about it, but the status line should not imply otherwise.
    txPowerSet = false;
  } else {
    txPowerSet = true;
  }

  if (esp_now_init() != ESP_OK) return false;
  if (esp_now_register_recv_cb(onRecv) != ESP_OK) {
    esp_now_deinit();
    return false;
  }
  if (!startPeer()) {
    esp_now_deinit();
    return false;
  }

  ready_ = true;
  return retune(net);
}

bool FastRadio::retune(const FastNet& net) {
  if (!net.valid) return false;
  return retuneTo(net.wifiChannel);
}

bool FastRadio::retuneTo(uint8_t channel) {
  // Twelve to fourteen are not legal everywhere we ship, and a board that sets
  // one goes deaf with no error reported anywhere.
  if (channel < 1 || channel > 11) return false;
  if (channel == channel_) return true;
  if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  channel_ = channel;
  currentChannel = channel;
  return true;
}

void FastRadio::end() {
  if (!ready_) return;
  esp_now_unregister_recv_cb();
  esp_now_deinit();
  ready_ = false;
  channel_ = 0;
}

bool FastRadio::send(const uint8_t* buf, size_t len) {
  if (!ready_ || buf == nullptr || len == 0 || len > FRAME_MAX) return false;
  esp_err_t err = esp_now_send(BROADCAST, buf, len);
  if (err != ESP_OK) {
    // ESP_ERR_ESPNOW_NO_MEM here means the driver's own transmit queue is
    // full, which is what saturation looks like from this side. Counted
    // rather than ignored so a board that has gone quiet can say so.
    sendFailCount++;
    lastSendErr = (int)err;
    return false;
  }
  return true;
}

bool FastRadio::poll(FastRx& out) {
  if (rxQueue == nullptr) return false;
  return xQueueReceive(rxQueue, &out, 0) == pdTRUE;
}

uint32_t FastRadio::dropped() const { return dropCount; }

int8_t FastRadio::txPowerDbm() const {
  int8_t q = 0;
  if (esp_wifi_get_max_tx_power(&q) != ESP_OK) return 0;
  return (int8_t)(q / 4); // quarter-dBm on the wire, whole dBm for a human
}

uint32_t FastRadio::sendFailed() const { return sendFailCount; }

int FastRadio::lastSendError() const { return lastSendErr; }

} // namespace touge
