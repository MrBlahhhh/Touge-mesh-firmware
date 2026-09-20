#include "hop.h"

namespace touge {

const uint8_t HOP_CHANNELS[FAST_CHANNELS] = {1, 6, 11};

bool hopNewer(uint8_t a, uint8_t b) {
  a &= HOP_GEN_MASK;
  b &= HOP_GEN_MASK;
  if (a == b) return false;
  // Serial number arithmetic. Straight comparison would make generation 0 look
  // older than 63 forever, so the ride would stop following hops the first
  // time the counter wrapped and every car would be stuck a generation behind.
  return (uint8_t)((a - b) & HOP_GEN_MASK) < HOP_GEN_HALF;
}

uint8_t hopPack(uint8_t index, uint8_t generation) {
  return (uint8_t)(((index & 0x03) << 6) | (generation & HOP_GEN_MASK));
}

void hopUnpack(uint8_t packed, uint8_t& index, uint8_t& generation) {
  index = (uint8_t)((packed >> 6) & 0x03);
  generation = (uint8_t)(packed & HOP_GEN_MASK);
}

void Hop::begin(uint8_t seed) {
  index_ = (uint8_t)(seed % FAST_CHANNELS);
  generation_ = 0;
  scan_ = index_;
}

bool Hop::observe(uint8_t packed) {
  uint8_t index = 0;
  uint8_t gen = 0;
  hopUnpack(packed, index, gen);
  // Two bits carry four values and there are three channels, so a corrupted or
  // hostile byte can name one that does not exist. The tag should have caught
  // it, but a bad index here would index off the end of the table.
  if (index >= FAST_CHANNELS) return false;
  if (!hopNewer(gen, generation_)) return false;

  bool moved = index != index_;
  index_ = index;
  generation_ = gen;
  scan_ = index;
  return moved;
}

void Hop::advance() {
  index_ = (uint8_t)((index_ + 1) % FAST_CHANNELS);
  generation_ = (uint8_t)((generation_ + 1) & HOP_GEN_MASK);
  scan_ = index_;
}

uint8_t Hop::scanNext() {
  scan_ = (uint8_t)((scan_ + 1) % FAST_CHANNELS);
  return HOP_CHANNELS[scan_];
}

} // namespace touge
