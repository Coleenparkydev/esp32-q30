/*
 * LILYGO LoRa32 (T3 v2.1)  ->  Soundcore Life Q30  (Bluetooth A2DP)
 * SD-card music player with on-board OLED.
 *
 *  - Plays every .mp3 on the microSD card, auto-advancing, looping forever.
 *  - OLED shows: Bluetooth status, volume %, track number, and song title.
 *  - The on-board BOOT button cycles the volume (no extra parts needed).
 *
 * Songs must be MP3, 44.1 kHz, stereo.
 *
 * Libraries (installed by the GitHub Action):
 *   pschatzmann/arduino-audio-tools (v1.2.5), ESP32-A2DP, arduino-libhelix,
 *   Adafruit SSD1306 (+ Adafruit GFX, Adafruit BusIO). SD/SPI/Wire come with the core.
 */

#include "SPI.h"
#include "SD.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "AudioTools.h"
#include "AudioTools/Communication/A2DPStream.h"
#include "AudioTools/Disk/AudioSourceSD.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"

// ---------- SETTINGS ----------
const char* BT_DEVICE_NAME = "Soundcore Life Q30";   // your headphones' exact name

// SD (TF) pins - LoRa32 T3 v2.1
#define SD_SCK   14
#define SD_MISO   2
#define SD_MOSI  15
#define SD_CS    13
#define LORA_CS  18                 // LoRa radio CS: held HIGH (deselected)

// OLED (SSD1306, I2C) - LoRa32 T3 v2.1
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST 16
#define OLED_ADDR 0x3C
#define SCREEN_W 128
#define SCREEN_H 64

// Built-in BOOT button cycles the volume
#define BTN_VOL  0
const float VOL_STEPS[] = {0.25f, 0.40f, 0.55f, 0.70f, 0.85f, 1.0f};
const int   VOL_COUNT   = 6;
int   volIdx = 3;                   // start at 0.70
// ------------------------------

SPIClass sdSPI(HSPI);
AudioSourceSD source("/", ".mp3", SD_CS, sdSPI);
MP3DecoderHelix decoder;
BufferRTOS<uint8_t> a2dpBuffer(0);
QueueStream<uint8_t> out(a2dpBuffer);
AudioPlayer player(source, out, decoder);
BluetoothA2DPSource a2dp;
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

bool oledOK = false;
long totalTracks = 0;

// A2DP pulls decoded PCM out of our buffer
int32_t get_data(uint8_t* data, int32_t bytes) {
  return a2dpBuffer.readArray(data, bytes);
}

// "/03_A cruel angels thesis.mp3"  ->  "A cruel angels thesis"
String cleanTitle(const char* raw) {
  if (raw == nullptr) return String("...");
  String s = String(raw);
  int slash = s.lastIndexOf('/');
  if (slash >= 0) s = s.substring(slash + 1);
  if (s.length() > 3 && isDigit(s[0]) && isDigit(s[1]) && s[2] == '_') s = s.substring(3);
  if (s.endsWith(".mp3") || s.endsWith(".MP3")) s = s.substring(0, s.length() - 4);
  return s;
}

void drawScreen() {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // top: BT status (left) + volume (right)
  display.setCursor(0, 0);
  display.print(a2dp.is_connected() ? "Q30" : "...");
  display.setCursor(72, 0);
  display.print("Vol ");
  display.print((int)(VOL_STEPS[volIdx] * 100));
  display.print("%");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // track number
  display.setCursor(0, 16);
  display.print("Track ");
  display.print((int)source.index() + 1);
  if (totalTracks > 0) { display.print("/"); display.print(totalTracks); }

  // song title (wrapped)
  display.setTextWrap(true);
  display.setCursor(0, 30);
  display.print(cleanTitle(source.toStr()));

  display.display();
}

void handleVolumeButton() {
  static bool prev = HIGH;
  static uint32_t lastPress = 0;
  bool now = digitalRead(BTN_VOL);
  if (prev == HIGH && now == LOW && millis() - lastPress > 250) {
    volIdx = (volIdx + 1) % VOL_COUNT;
    player.setVolume(VOL_STEPS[volIdx]);
    lastPress = millis();
    drawScreen();
  }
  prev = now;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  pinMode(BTN_VOL, INPUT_PULLUP);

  // OLED init
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW); delay(20); digitalWrite(OLED_RST, HIGH);
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.println("Reading SD card");
    display.display();
  }

  // SD SPI bus on the TF-card pins
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  // PCM buffer between decoder and A2DP
  a2dpBuffer.resize(24 * 1024);
  out.begin(95);

  player.setDelayIfOutputFull(0);
  player.setVolume(VOL_STEPS[volIdx]);
  player.begin();
  player.setAutoNext(true);
  totalTracks = source.size();       // cache once (slow to recompute)

  if (oledOK) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connecting to:");
    display.println(BT_DEVICE_NAME);
    display.display();
  }

  a2dp.set_data_callback(get_data);
  a2dp.start(BT_DEVICE_NAME);
}

void loop() {
  player.copy();
  if (!player.isActive()) player.setIndex(0);   // end of playlist -> loop to song 0

  handleVolumeButton();

  static uint32_t lastUI = 0;
  static int lastIdx = -1;
  int idx = (int)source.index();
  if (idx != lastIdx || millis() - lastUI > 1000) {
    lastIdx = idx;
    lastUI = millis();
    drawScreen();
  }
}
