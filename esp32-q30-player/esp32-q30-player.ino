/*
 * LILYGO LoRa32 (T3 v2.1) -> Soundcore Life Q30 (Bluetooth A2DP)
 * SD-card music player with auto-detecting OLED + BOOT-button volume.
 *
 * Robust OLED init: probes the I2C bus for the display address/pins. If the
 * OLED is missing or at a different address, it is skipped (music still plays)
 * instead of hanging the boot. Serial @115200 prints every step.
 *
 * Songs must be MP3, 44.1 kHz, stereo.
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
const char* BT_DEVICE_NAME = "Soundcore Life Q30";

// SD (TF) pins - LoRa32 T3 v2.1
#define SD_SCK   14
#define SD_MISO   2
#define SD_MOSI  15
#define SD_CS    13
#define LORA_CS  18

// OLED
#define SCREEN_W 128
#define SCREEN_H 64

// Built-in BOOT button cycles the volume
#define BTN_VOL  0
const float VOL_STEPS[] = {0.25f, 0.40f, 0.55f, 0.70f, 0.85f, 1.0f};
const int   VOL_COUNT   = 6;
int   volIdx = 3;
// ------------------------------

SPIClass sdSPI(HSPI);
AudioSourceSD source("/", ".mp3", SD_CS, sdSPI);
MP3DecoderHelix decoder;
BufferRTOS<uint8_t> a2dpBuffer(0);
QueueStream<uint8_t> out(a2dpBuffer);
AudioPlayer player(source, out, decoder);
BluetoothA2DPSource a2dp;
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

bool   oledOK = false;
uint8_t oledAddr = 0;
long   totalTracks = 0;

int32_t get_data(uint8_t* data, int32_t bytes) {
  return a2dpBuffer.readArray(data, bytes);
}

// quick, non-blocking probe of one pin pair for a 0x3C / 0x3D display
bool probeI2C(int sda, int scl, uint8_t &foundAddr) {
  Wire.end(); delay(20);
  Wire.begin(sda, scl);
  Wire.setClock(400000);
  Wire.setTimeOut(50);
  delay(20);
  uint8_t cand[2] = {0x3C, 0x3D};
  for (int i = 0; i < 2; i++) {
    Wire.beginTransmission(cand[i]);
    if (Wire.endTransmission() == 0) { foundAddr = cand[i]; return true; }
  }
  return false;
}

void initOLED() {
  if (probeI2C(21, 22, oledAddr)) {
    Serial.printf("OLED found 0x%02X on SDA=21 SCL=22\n", oledAddr);
  } else if (probeI2C(4, 15, oledAddr)) {
    Serial.printf("OLED found 0x%02X on SDA=4 SCL=15\n", oledAddr);
  } else {
    Serial.println("No OLED found - running without display");
    oledOK = false;
    return;
  }
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, oledAddr);
  Serial.printf("display.begin() -> %s\n", oledOK ? "OK" : "FAIL");
}

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
  display.setCursor(0, 0);
  display.print(a2dp.is_connected() ? "Q30" : "...");
  display.setCursor(72, 0);
  display.print("Vol ");
  display.print((int)(VOL_STEPS[volIdx] * 100));
  display.print("%");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setCursor(0, 16);
  display.print("Track ");
  display.print((int)source.index() + 1);
  if (totalTracks > 0) { display.print("/"); display.print(totalTracks); }
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
  delay(500);
  Serial.println("\n\n=== LoRa32 SD player (OLED) ===");

  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  pinMode(BTN_VOL, INPUT_PULLUP);

  Serial.println("Init OLED...");
  initOLED();
  if (oledOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.println("Reading SD card");
    display.display();
  }

  Serial.println("Init SD bus...");
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  Serial.println("Start player...");
  a2dpBuffer.resize(24 * 1024);
  out.begin(95);
  player.setDelayIfOutputFull(0);
  player.setVolume(VOL_STEPS[volIdx]);
  player.begin();
  player.setAutoNext(true);
  totalTracks = source.size();
  Serial.printf("Tracks found: %ld\n", totalTracks);

  if (oledOK) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Connecting to:");
    display.println(BT_DEVICE_NAME);
    display.display();
  }

  Serial.println("Start A2DP...");
  a2dp.set_data_callback(get_data);
  a2dp.start(BT_DEVICE_NAME);
  Serial.println("setup() done");
}

void loop() {
  player.copy();
  if (!player.isActive()) player.setIndex(0);

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
