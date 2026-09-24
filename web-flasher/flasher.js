// Touge radio flasher. Talks to the radio over Web Serial with esptool-js,
// works out which Heltec it is from the chip, and writes a build listed in
// releases.json (made by publish-release.ps1).

// Pinned to an exact version: a flasher that changes under us between two
// visits is the last thing that should happen to a radio.
import { ESPLoader, Transport } from "https://cdn.jsdelivr.net/npm/esptool-js@0.7.0/bundle.js";

const BAUD_FLASH = 921600;
const BAUD_ROM = 115200;

// Same table as flash-all.ps1. The USB ID is never used: it only says who made
// the USB bridge, and every other ESP32 on the bench has one of the same two.
const BOARDS = {
  "heltec-v3": {
    name: "Heltec V3",
    chip: "ESP32-S3",
    flashMB: 8,
    psramMB: 0,
    spec: "ESP32-S3 · 8 MB flash · no PSRAM",
    blurb: "Works. 2.4 GHz is on a fixed spring antenna with no connector.",
    art: boardArt("V3", false),
  },
  "heltec-v4": {
    name: "Heltec V4",
    chip: "ESP32-S3",
    flashMB: 16,
    psramMB: 2,
    spec: "ESP32-S3 · 16 MB flash · 2 MB PSRAM",
    blurb: "Recommended, in the 28 dBm high-power version. Has a u.FL 2.4G antenna connector.",
    tag: "Recommended",
    art: boardArt("V4", true),
  },
};

function boardForChip(chip) {
  if (chip.name !== "ESP32-S3") return null;
  if (chip.flashMB === 8 && chip.psramMB === 0) return "heltec-v3";
  if (chip.flashMB === 16 && chip.psramMB === 2) return "heltec-v4";
  return null;
}

// ---------------------------------------------------------------- state

const state = {
  manifest: null,
  release: null,
  boardKey: null,
  pickedManually: false,
  chip: null,          // { name, flashMB, psramMB, mac } once read
  port: null,
  transport: null,
  loader: null,
  busy: false,
};

const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------- log

const logEl = $("log");
function log(line) {
  logEl.textContent += line + "\n";
  logEl.scrollTop = logEl.scrollHeight;
}
const terminal = {
  clean() {},
  writeLine(data) { log(data); },
  write(data) { logEl.textContent += data; },
};

// ---------------------------------------------------------------- serial

async function openLoader() {
  if (state.loader) return state.loader;
  // No USB filter on purpose: the chip check decides what this is, not the
  // bridge vendor, so every port is offered.
  state.port = await navigator.serial.requestPort();
  state.transport = new Transport(state.port, false);
  state.loader = new ESPLoader({
    transport: state.transport,
    baudrate: BAUD_FLASH,
    romBaudrate: BAUD_ROM,
    terminal,
  });
  try {
    // Resets into the ROM bootloader, identifies the chip, loads the stub and
    // moves to the fast baud rate.
    await state.loader.main();
  } catch (err) {
    await closeLoader();
    throw err;
  }
  state.port.addEventListener("disconnect", onUnplugged);
  return state.loader;
}

async function closeLoader() {
  const transport = state.transport;
  if (state.port) state.port.removeEventListener("disconnect", onUnplugged);
  state.loader = null;
  state.transport = null;
  state.port = null;
  if (transport) {
    try { await transport.disconnect(); } catch { /* already gone */ }
  }
}

function onUnplugged() {
  state.loader = null;
  state.transport = null;
  state.port = null;
  state.chip = null;
  if (!state.pickedManually) state.boardKey = null;
  log("Radio unplugged.");
  render();
}

async function readChip(loader) {
  const flashSize = await loader.detectFlashSize();           // "8MB", "16MB" or undefined
  const features = await loader.chip.getChipFeatures(loader);  // includes "Embedded PSRAM 2MB (AP_3v3)"
  const psram = features.join(" ").match(/Embedded PSRAM (\d+)MB/);
  return {
    name: loader.chip.CHIP_NAME,
    flashMB: flashSize ? parseInt(flashSize, 10) : 0,
    psramMB: psram ? parseInt(psram[1], 10) : 0,
    mac: await loader.chip.readMac(loader),
  };
}

// ---------------------------------------------------------------- step 1: device

async function connectAndDetect() {
  if (state.busy) return;
  state.busy = true;
  $("device-error").hidden = true;
  render();
  try {
    await closeLoader();
    const loader = await openLoader();
    state.chip = await readChip(loader);
    const detected = boardForChip(state.chip);
    log(`Chip ${state.chip.name}, ${state.chip.flashMB} MB flash, ${state.chip.psramMB} MB PSRAM -> ${detected || "unknown"}`);
    if (detected) {
      state.boardKey = detected;
      state.pickedManually = false;
    } else if (!state.pickedManually) {
      state.boardKey = null;
      showDeviceError(`Not a Heltec V3 or V4 (${chipText(state.chip)}). Nothing flashed. If you're sure what it is, use "I know what this is".`);
      await closeLoader();
    }
  } catch (err) {
    if (err && err.name === "NotFoundError") {
      log("No port picked.");
    } else {
      showDeviceError(`Couldn't talk to the radio: ${messageOf(err)}. See "If it won't connect" below.`);
      log(String(err && err.stack || err));
    }
  } finally {
    state.busy = false;
    render();
  }
}

function showDeviceError(text) {
  $("device-error").textContent = text;
  $("device-error").hidden = false;
}

function chipText(chip) {
  return `${chip.name}, ${chip.flashMB || "?"} MB flash, ${chip.psramMB ? chip.psramMB + " MB" : "no"} PSRAM`;
}

function openDevicePicker() {
  const grid = $("device-grid");
  grid.textContent = "";
  for (const [key, board] of Object.entries(BOARDS)) {
    const card = document.createElement("button");
    card.type = "button";
    card.className = "device-card";
    card.innerHTML = `${board.art}
      <span class="name">${board.name}</span>
      ${board.tag ? `<span class="tag">${board.tag}</span>` : ""}
      <span class="spec">${board.spec}</span>
      <span class="blurb">${board.blurb}</span>`;
    card.addEventListener("click", () => {
      state.boardKey = key;
      state.pickedManually = true;
      $("device-error").hidden = true;
      $("device-dialog").close();
      render();
    });
    grid.appendChild(card);
  }
  $("device-dialog").showModal();
}

// ---------------------------------------------------------------- step 2: firmware

async function loadReleases() {
  const select = $("release-select");
  try {
    const response = await fetch("releases.json", { cache: "no-cache" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    state.manifest = await response.json();
  } catch (err) {
    select.innerHTML = "<option>No builds published</option>";
    $("release-detail").textContent = `Couldn't load releases.json (${messageOf(err)}).`;
    return;
  }
  // Newest build first, and the newest is the default.
  const releases = [...state.manifest.releases].sort((a, b) => b.build - a.build);
  state.manifest.releases = releases;
  select.textContent = "";
  releases.forEach((release, i) => {
    const option = document.createElement("option");
    option.value = String(i);
    option.textContent = `Build ${release.build}${i === 0 ? " (latest)" : ""} · ${release.date}${release.bench ? " · bench" : ""}`;
    select.appendChild(option);
  });
  select.disabled = releases.length === 0;
  state.release = releases[0] || null;
  select.addEventListener("change", () => {
    state.release = state.manifest.releases[Number(select.value)];
    render();
  });
  render();
}

// ---------------------------------------------------------------- step 3: flash

function openFlashDialog() {
  const board = BOARDS[state.boardKey];
  const release = state.release;
  $("flash-art").innerHTML = board.art;
  $("flash-what").innerHTML = `<strong>${board.name}</strong>, Touge build ${release.build}<br>
    <span class="note">Meshtastic ${release.meshtasticVersion}${state.pickedManually ? " · board picked by hand" : ""}</span>`;
  $("progress-wrap").hidden = true;
  $("result").hidden = true;
  $("flash-choices").querySelectorAll("input").forEach((input) => { input.disabled = false; });
  $("flash-choices").querySelector("input[value=update]").checked = true;
  syncStartButton();
  $("btn-start").disabled = false;
  $("flash-dialog").showModal();
}

function selectedMode() {
  return $("flash-choices").querySelector("input:checked").value;
}

function syncStartButton() {
  const fresh = selectedMode() === "fresh";
  const button = $("btn-start");
  button.textContent = fresh ? "Erase and install" : "Update now";
  button.classList.toggle("btn-danger", fresh);
  button.classList.toggle("btn-primary", !fresh);
}

function setStage(text, fraction) {
  $("progress-wrap").hidden = false;
  $("progress-stage").textContent = text;
  const indeterminate = fraction === null;
  $("progress").classList.toggle("indeterminate", indeterminate);
  $("progress-bar").style.width = indeterminate ? "" : `${Math.round(fraction * 100)}%`;
  $("progress-pct").textContent = indeterminate ? "" : `${Math.round(fraction * 100)}%`;
}

function showResult(ok, html) {
  const box = $("result");
  box.className = `result ${ok ? "result-ok" : "result-bad"}`;
  box.innerHTML = html;
  box.hidden = false;
}

// Refusals that stop before anything is written. Thrown, caught in startFlash
// and shown as "nothing flashed".
class Refusal extends Error {}

async function startFlash() {
  if (state.busy) return;
  state.busy = true;
  const mode = selectedMode();
  const boardKey = state.boardKey;
  const release = state.release;
  const plan = release.boards[boardKey];
  let wroteSomething = false;
  $("btn-start").disabled = true;
  $("flash-choices").querySelectorAll("input").forEach((input) => { input.disabled = true; });
  $("result").hidden = true;
  render();

  try {
    if (!plan) throw new Refusal(`Build ${release.build} has no image for the ${BOARDS[boardKey].name}.`);

    setStage("Connecting", null);
    const loader = await openLoader();
    state.chip = await readChip(loader);
    checkChipAgainstPick(boardKey);

    // Update: the app at 0x10000, then otadata at 0xe000 pointing the
    // bootloader at app0 (a radio can be left booting app1 by Meshtastic's
    // OTA loader; publish-release.ps1 has the details). NVS at 0x9000, which
    // holds the Bluetooth bond, and the file system with the settings are not
    // in the list, so they survive.
    // Fresh: erase everything, then the factory image at 0x0 (bootloader,
    // partition table, otadata and app) and the file system image if the
    // build has one.
    const files = mode === "fresh" ? plan.fresh : plan.update;

    if (mode === "update") {
      setStage("Checking the partition table", null);
      await checkPartitionTable(loader, plan.partitionTable);
    }

    // Download and check everything before touching the flash, so a bad
    // download can never leave a half-written radio.
    const images = [];
    for (const [i, file] of files.entries()) {
      setStage(`Downloading ${shortName(file)}`, i / files.length);
      images.push(await download(file));
    }

    if (mode === "fresh") {
      setStage("Erasing the whole flash (up to a minute)", null);
      wroteSomething = true;
      await loader.eraseFlash();
    }

    const totalBytes = images.reduce((sum, image) => sum + image.data.length, 0);
    const doneBefore = images.map((_, i) => images.slice(0, i).reduce((sum, image) => sum + image.data.length, 0));
    setStage("Writing", 0);
    wroteSomething = true;
    await loader.writeFlash({
      fileArray: images.map((image) => ({ data: image.data, address: image.address })),
      flashMode: "keep",
      flashFreq: "keep",
      flashSize: "keep",
      eraseAll: false,
      compress: true,
      reportProgress(fileIndex, written, total) {
        const fileFraction = total ? written / total : 1;
        const overall = (doneBefore[fileIndex] + fileFraction * images[fileIndex].data.length) / totalBytes;
        setStage(`Writing ${shortName(files[fileIndex])}`, overall);
      },
    });

    for (const [i, image] of images.entries()) {
      setStage(`Verifying ${shortName(files[i])}`, null);
      const onFlash = await loader.flashMd5sum(image.address, image.data.length);
      log(`md5 at 0x${image.address.toString(16)}: flash ${onFlash}, expected ${files[i].md5}`);
      if (onFlash !== files[i].md5) {
        throw new Error(`What's on the flash at 0x${image.address.toString(16)} doesn't match ${shortName(files[i])}. Flash again; if it keeps failing, try another cable or port.`);
      }
    }

    setStage("Restarting the radio", 1);
    await loader.after("hard_reset");
    await closeLoader();
    state.chip = null;
    setStage("Done", 1);
    showResult(true, mode === "fresh"
      ? `Build ${release.build} installed and verified. Next:<ol>
          <li>On the phone, forget the old pairing for this radio in Bluetooth settings.</li>
          <li>In the Touge app, pick the radio and pair with PIN <code>123456</code>.</li>
          <li>Join the ride in the app so the radio gets the ride key.</li></ol>`
      : `Build ${release.build} written and verified. The radio keeps its pairing and ride, so it's ready once it has restarted.`);
  } catch (err) {
    log(String(err && err.stack || err));
    const nothing = err instanceof Refusal || !wroteSomething;
    const reason = messageOf(err);
    showResult(false, nothing
      ? `Nothing flashed. ${escapeHtml(reason)}`
      : `Flashing failed partway: ${escapeHtml(reason)} The radio may not start until it's flashed again. Unplug it, plug it back in and run it again; if it won't connect, hold BOOT while plugging in.`);
    setStage(nothing ? "Stopped" : "Failed", null);
    $("progress").classList.remove("indeterminate");
    await closeLoader();
  } finally {
    state.busy = false;
    $("flash-choices").querySelectorAll("input").forEach((input) => { input.disabled = false; });
    $("btn-start").disabled = false;
    render();
  }
}

// The chip is read again right before writing. An automatic pick has to still
// match; a hand pick is trusted like flash-all.ps1's -Map, except that the
// image must fit and must be for this chip family.
function checkChipAgainstPick(boardKey) {
  const board = BOARDS[boardKey];
  const detected = boardForChip(state.chip);
  log(`Chip now: ${chipText(state.chip)}`);
  if (state.chip.name !== board.chip) {
    throw new Refusal(`This is an ${state.chip.name}, and the ${board.name} image is for an ${board.chip}.`);
  }
  if (state.chip.flashMB && state.chip.flashMB < board.flashMB) {
    throw new Refusal(`This chip has ${state.chip.flashMB} MB of flash and the ${board.name} image needs ${board.flashMB} MB.`);
  }
  if (!state.pickedManually && detected !== boardKey) {
    throw new Refusal(detected
      ? `The radio on this port is a ${BOARDS[detected].name}, not the ${board.name} picked earlier. Press Connect again.`
      : `The radio on this port isn't a Heltec V3 or V4 (${chipText(state.chip)}).`);
  }
}

// Update writes the app into the existing layout, so the layout on the radio
// has to be the one this build was linked for. The table sits at 0x8000; an
// md5 of it is a single cheap command. A radio that fails this needs a fresh
// install, which writes the table as part of the factory image.
async function checkPartitionTable(loader, table) {
  const offset = parseInt(table.offset, 16);
  const onFlash = await loader.flashMd5sum(offset, table.size);
  log(`Partition table md5: flash ${onFlash}, build ${table.md5}`);
  if (onFlash !== table.md5) {
    throw new Refusal("The partition layout on this radio isn't the one this build uses, so an update could leave it unbootable. Use Fresh install instead.");
  }
}

async function download(file) {
  const response = await fetch(file.path);
  if (!response.ok) throw new Refusal(`Couldn't download ${file.path} (HTTP ${response.status}).`);
  const data = new Uint8Array(await response.arrayBuffer());
  if (data.length !== file.size) {
    throw new Refusal(`${file.path} is ${data.length} bytes, the manifest says ${file.size}.`);
  }
  // The ROM stub only does md5, which checks the flash afterwards. sha256 is
  // what the browser can do, so it checks the download before any write.
  const digest = new Uint8Array(await crypto.subtle.digest("SHA-256", data));
  const sha256 = [...digest].map((b) => b.toString(16).padStart(2, "0")).join("");
  if (sha256 !== file.sha256) throw new Refusal(`${file.path} failed its checksum. Reload the page and try again.`);
  log(`Downloaded ${file.path}, ${data.length} bytes, sha256 ok`);
  return { data, address: parseInt(file.offset, 16) };
}

function shortName(file) {
  return { app: "firmware", otadata: "boot selector", factory: "full image", littlefs: "file system" }[file.what] || file.what;
}

// ---------------------------------------------------------------- render

function render() {
  const board = state.boardKey ? BOARDS[state.boardKey] : null;
  const release = state.release;
  const connected = !!state.loader;

  $("step-device").classList.toggle("ready", !!board);
  $("device-picked").hidden = !board;
  if (board) {
    $("device-picked-art").innerHTML = board.art;
    $("device-picked-name").textContent = board.name;
    $("device-picked-how").textContent = state.pickedManually ? "Picked by hand" : "Detected from the chip";
    $("device-hint").hidden = true;
  } else {
    $("device-hint").hidden = false;
  }
  $("chip-line").hidden = !state.chip;
  if (state.chip) $("chip-line").textContent = `${chipText(state.chip)} · ${state.chip.mac}`;
  $("btn-connect").disabled = state.busy || !("serial" in navigator);
  $("btn-connect").textContent = state.busy && !$("flash-dialog").open ? "Connecting..." : connected ? "Connect again" : "Connect";

  $("step-firmware").classList.toggle("ready", !!release);
  $("bench-banner").hidden = !(release && release.bench);
  if (release) {
    $("release-detail").textContent = `Meshtastic ${release.meshtasticVersion}. ${release.notes || ""}`.trim();
    const repo = state.manifest.sourceRepo;
    const meshtastic = `<a href="https://github.com/meshtastic/firmware/tree/${release.meshtasticCommit}" target="_blank" rel="noopener">meshtastic/firmware@${release.meshtasticCommit.slice(0, 7)}</a>`;
    const sourceHtml = release.sourceCommit
      ? `Source: <a href="${repo}/tree/${release.sourceCommit}" target="_blank" rel="noopener">Touge-mesh-firmware@${release.sourceCommit.slice(0, 7)}</a> on ${meshtastic}.`
      : `Source not published for this bench build. Base: ${meshtastic}.`;
    $("release-source").innerHTML = sourceHtml;
    $("source-exact").innerHTML = `Build ${release.build}: ${sourceHtml}`;
    if (board && !release.boards[state.boardKey]) {
      $("release-detail").textContent += ` Not built for the ${board.name}.`;
    }
  }

  const canFlash = !!board && !!release && !!release.boards[state.boardKey] && ("serial" in navigator) && !state.busy;
  $("btn-flash").disabled = !canFlash;
  $("step-flash").classList.toggle("ready", canFlash);
  $("flash-hint").textContent = canFlash
    ? `Build ${release.build} for the ${board.name}. You'll choose between Update and Fresh install next.`
    : "Pick a device and a build first.";
}

// ---------------------------------------------------------------- helpers

function messageOf(err) {
  return (err && err.message) || String(err);
}

function escapeHtml(text) {
  return text.replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
}

// A plain drawing of a Heltec-style board: PCB, OLED, LoRa u.FL, USB-C, and
// on the V4 the extra 2.4G u.FL. Not a photo and not anyone's artwork.
function boardArt(label, hasWifiConnector) {
  return `<svg viewBox="0 0 160 100" role="img" aria-label="Heltec ${label}">
    <rect x="4" y="10" width="152" height="80" rx="8" fill="#1f2a37" stroke="#3b4a5c" stroke-width="2"/>
    <rect x="28" y="24" width="70" height="44" rx="3" fill="#0b1220" stroke="#4b5563"/>
    <rect x="34" y="30" width="58" height="32" rx="1" fill="#0f2233"/>
    <text x="63" y="51" text-anchor="middle" font-family="system-ui, sans-serif" font-size="14" font-weight="700" fill="#ff9a3c">${label}</text>
    <rect x="0" y="40" width="14" height="20" rx="3" fill="#9ca3af"/>
    <circle cx="140" cy="26" r="6" fill="#d4a017" stroke="#8a6a10"/>
    <text x="140" y="44" text-anchor="middle" font-family="system-ui, sans-serif" font-size="8" fill="#9ca3af">LoRa</text>
    ${hasWifiConnector ? `<circle cx="140" cy="66" r="6" fill="#d4a017" stroke="#8a6a10"/>
    <text x="140" y="84" text-anchor="middle" font-family="system-ui, sans-serif" font-size="8" fill="#9ca3af">2.4G</text>` : ""}
    <rect x="108" y="30" width="16" height="16" rx="2" fill="#374151"/>
    <rect x="108" y="54" width="16" height="10" rx="2" fill="#374151"/>
  </svg>`;
}

// ---------------------------------------------------------------- wire up

function init() {
  if (!("serial" in navigator)) $("no-serial").hidden = false;
  $("btn-connect").addEventListener("click", connectAndDetect);
  $("btn-manual").addEventListener("click", openDevicePicker);
  $("btn-flash").addEventListener("click", openFlashDialog);
  $("btn-start").addEventListener("click", startFlash);
  $("flash-choices").addEventListener("change", syncStartButton);
  // The flash dialog stays open while busy: closing it mid-write would hide
  // the only progress there is.
  document.querySelectorAll("dialog [data-close]").forEach((button) => {
    button.addEventListener("click", () => {
      const dialog = button.closest("dialog");
      if (dialog.id === "flash-dialog" && state.busy) return;
      dialog.close();
    });
  });
  $("flash-dialog").addEventListener("cancel", (event) => { if (state.busy) event.preventDefault(); });
  render();
  loadReleases();
}

init();
