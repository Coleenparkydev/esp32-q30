# ESP32 → Soundcore Life Q30 one-song player

Plays a single MP3 stored in the ESP32's own flash to your Q30 over Bluetooth.
No SD card. No software installed on the Chromebook — build runs on GitHub, flash
runs in the Chrome browser.

**Your hardware (confirmed):** ESP32-D0WD-V3, 4 MB flash, no PSRAM, WiFi + BT.

---

## How it works

The board has two modes, chosen automatically at power-on:

- **Upload mode** — if no song is stored yet, *or* you hold the **BOOT** button while
  powering on. The board makes a WiFi network called `ESP32_Music`. You connect to it,
  open `http://192.168.4.1`, and upload one `.mp3`. It saves the file and reboots.
- **Play mode** — if a song is stored. WiFi turns **off**, Bluetooth turns **on**, the
  board connects to your Q30 by name and loops the song forever.

WiFi and Bluetooth never run together, so there's no antenna conflict.

---

## Step 1 — Convert your song to 96 kbps

192 kbps is too big for 4 MB flash. Target: **MP3, 96 kbps, 44.1 kHz, stereo**, and
keep the song **under ~2.7 minutes** (≈ 2 MB) so it fits the 2 MB storage partition.
Longer song? Use 64 kbps (fits ~4 min) or trim it.

Easiest on a Chromebook (no install): any browser MP3 bitrate converter
(e.g. search "online audio converter", set bitrate to 96 kbps). Or ask me to build you
a tiny in-browser converter so your music never leaves the laptop.

Name the result whatever you like — you'll upload it through the web page later.

---

## Step 2 — Put this repo on GitHub

1. Create a new repo on github.com (e.g. `esp32-q30`).
2. Upload these files keeping the folder layout:
   ```
   esp32-q30/
   ├─ .github/workflows/build.yml
   └─ esp32-q30-player/
      ├─ esp32-q30-player.ino
      └─ partitions.csv
   ```
   (You can drag-and-drop folders in the GitHub web "Add file → Upload files" screen.)

## Step 3 — Let GitHub build it

- Push triggers the build automatically. Or go to the **Actions** tab → **build** →
  **Run workflow**.
- When it finishes (~3–4 min), open the run → **Artifacts** → download **firmware**.
- Unzip it. You'll get three files:
  `*.ino.bootloader.bin`, `*.ino.partitions.bin`, `*.ino.bin`.

> Check the "Show app size" step in the log. If compile **fails with the app not fitting**,
> tell me — we shrink the storage partition a touch and give the app more room.

## Step 4 — Flash with esptool-js (the tool you already have open)

At https://espressif.github.io/esptool-js/ , connect to the board, then in the
**Program** section add three rows with these **flash addresses**:

| Flash Address | File |
|---|---|
| `0x1000`  | `esp32-q30-player.ino.bootloader.bin` |
| `0x8000`  | `esp32-q30-player.ino.partitions.bin` |
| `0x10000` | `esp32-q30-player.ino.bin` |

Then click **Program**. Wait for "Hard resetting".

## Step 5 — Upload your song

1. After flashing, the board boots into **upload mode** (no song yet).
2. On the Chromebook WiFi list, connect to **`ESP32_Music`**.
3. Open **http://192.168.4.1** → choose your 96 kbps mp3 → **Upload & Play**.
4. The board saves it and reboots.

## Step 6 — Play

1. Turn the **Q30 on and put it in pairing mode** (not connected to your phone).
2. The board boots into play mode, finds "Soundcore Life Q30", connects, and plays —
   looping forever.

**To change the song later:** hold the **BOOT** button while powering the board on.
That forces upload mode again. Upload a new file, done.

---

## Tuning knobs (in `esp32-q30-player.ino`)

- `BT_DEVICE_NAME` — must match your headphones' Bluetooth name exactly.
- `AP_SSID` / `AP_PASS` — the upload-mode WiFi name/password.

## If something misbehaves

- **Won't connect to Q30:** make sure the headphones are discoverable and not paired to
  your phone. The name must match exactly.
- **Playback stutters:** the song's bitrate/sample rate is too heavy for the RAM (no
  PSRAM on this board). Re-export at 96 kbps, or try 32 kHz, or mono.
- **Compile fails on size:** ping me, we adjust `partitions.csv`.

This is v1 — the first cloud build is also our first real test. If the build log throws
a library/include error, send it over and we fix the pin/path together.
