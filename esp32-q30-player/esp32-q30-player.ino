/*
 * LILYGO LoRa32 (T3 v2.1)  ->  Soundcore Life Q30  (Bluetooth A2DP, SBC)
 * Multi-song player. Reads ALL .mp3 files from the on-board TF (microSD) slot,
 * plays them in order, auto-advances to the next song, and loops the whole
 * playlist forever.
 *
 * Songs must be: MP3, 44.1 kHz, stereo (192 kbps is fine).
 *   - 44.1 kHz is required: the A2DP link sends 44.1 kHz, so any other rate
 *     (e.g. 48 kHz or 32 kHz) plays at the wrong speed/pitch.
 *   - stereo is required: mono is interpreted as half-speed and sounds fast.
 *
 * No WiFi, no upload step. Put files on the SD card from your PC and power on.
 *
 * Libraries (installed by the GitHub Action, pinned):
 *   - pschatzmann/arduino-audio-tools  (v1.2.5)
 *   - pschatzmann/ESP32-A2DP
 *   - pschatzmann/arduino-libhelix
 *   SD + SPI come with the ESP32 Arduino core.
 */

#include "SPI.h"
#include "SD.h"
#include "AudioTools.h"
#include "AudioTools/Communication/A2DPStream.h"   // brings in BluetoothA2DPSource
#include "AudioTools/Disk/AudioSourceSD.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"

// ---------- SETTINGS ----------
const char* BT_DEVICE_NAME = "Soundcore Life Q30";  // your headphones' exact BT name

// LILYGO LoRa32 T3 v2.1 TF (microSD) card pins
#define SD_SCK   14
#define SD_MISO   2
#define SD_MOSI  15
#define SD_CS    13
// LoRa radio chip-select on this board: drive HIGH so the radio stays
// deselected and never disturbs anything while we use the SD card.
#define LORA_CS  18
// ------------------------------

SPIClass sdSPI(HSPI);                                // dedicated SPI bus for the SD card
AudioSourceSD source("/", ".mp3", SD_CS, sdSPI);     // index every .mp3 in the card root
MP3DecoderHelix decoder;
BufferRTOS<uint8_t> a2dpBuffer(0);
QueueStream<uint8_t> out(a2dpBuffer);
AudioPlayer player(source, out, decoder);
BluetoothA2DPSource a2dp;

// A2DP pulls decoded PCM out of our buffer
int32_t get_data(uint8_t* data, int32_t bytes) {
  return a2dpBuffer.readArray(data, bytes);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Keep the LoRa radio deselected (we are not using it)
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  // Bring up the SD SPI bus on the LoRa32 TF-card pins
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  // 20 KB PCM buffer between the decoder and the A2DP link
  a2dpBuffer.resize(20 * 1024);
  out.begin(95);                 // start feeding A2DP once the buffer is 95% full

  player.setDelayIfOutputFull(0);
  player.setVolume(0.7);
  player.begin();                // mounts SD, indexes .mp3 files, opens the first track
  player.setAutoNext(true);      // when a song ends, automatically start the next one

  a2dp.set_data_callback(get_data);
  a2dp.start(BT_DEVICE_NAME);    // scan for and connect to the Q30 by name
}

void loop() {
  player.copy();                 // decode + push PCM into the buffer
  if (!player.isActive()) {
    player.setIndex(0);          // reached the end of the playlist -> loop back to song 0
  }
}
