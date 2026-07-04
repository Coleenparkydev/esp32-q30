/*
 * LoRa32 T3 (v1.6.1 / v2.1) RFID Jukebox -> Soundcore Life Q30 (A2DP)
 *
 * Tap an RFID card = play that album. Each card's UID is a folder on the SD card.
 * Songs inside the album play SHUFFLED and loop. At the end of each song the reader
 * is checked; tapping a different card switches album at the next song boundary
 * (never mid-song). Volume is controlled from the headphones.
 *
 * SD-card layout (folders named by card UID, uppercase; songs 44.1kHz stereo mp3):
 *   /79C94EB6/  song1.mp3  song2.mp3 ...
 *   /69E589B6/  ...
 *   ... (one folder per card)
 *
 * Wiring (RC522 shares the SD SPI bus):
 *   SDA->IO4  SCK->IO14  MOSI->IO15  MISO->IO2  RST->3.3V  3.3V->3.3V  GND->GND  IRQ->nc
 */

#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "AudioTools.h"
#include "AudioTools/Communication/A2DPStream.h"
#include "AudioTools/Disk/AudioSourceSD.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"

// ---------- SETTINGS ----------
const char* BT_DEVICE_NAME = "Soundcore Life Q30";
#define RC522_CS 4
#define SD_CS   13
#define SD_SCK  14
#define SD_MISO  2
#define SD_MOSI 15
#define LORA_CS 18
#define MAX_SONGS 100
// Safety ceiling: the signal can never exceed this % of full scale.
// 100 = fully transparent (normal music untouched). Lower it (e.g. 90) for a
// stricter hard cap, at the cost of a little fidelity on the very loudest peaks.
#define LIMIT_PCT 100
// ------------------------------
static const int16_t SAFE_LIMIT = (int16_t)(32767L * LIMIT_PCT / 100);

MFRC522 mfrc(RC522_CS, MFRC522::UNUSED_PIN);          // CS=4, soft reset (RST tied to 3.3V)
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOK = false;

AudioSourceSD source("/", ".mp3", SD_CS);             // uses global SPI, CS=13
MP3DecoderHelix decoder;
BufferRTOS<uint8_t> a2dpBuffer(0);
QueueStream<uint8_t> out(a2dpBuffer);
AudioPlayer player(source, out, decoder);
BluetoothA2DPSource a2dp;

String currentAlbum = "";
String albumPath;                                     // persists for source.setPath()
int shuffleOrder[MAX_SONGS];
int songCount = 0;
int shufflePos = 0;

int32_t get_data(uint8_t* d, int32_t n) {
  int32_t got = a2dpBuffer.readArray(d, n);
  // Safety clamp: guarantees the samples sent to the headphones never exceed the
  // ceiling, so no bug/overflow can produce a sudden blast. Transparent at 100%.
  int16_t* s = (int16_t*)d;
  int count = got / 2;
  for (int i = 0; i < count; i++) {
    if (s[i] >  SAFE_LIMIT) s[i] =  SAFE_LIMIT;
    else if (s[i] < -SAFE_LIMIT) s[i] = -SAFE_LIMIT;
  }
  return got;
}

void initOLED() {
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Wire.setTimeOut(50);
  Wire.beginTransmission(0x3C);
  if (Wire.endTransmission() == 0) oledOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial.printf("OLED: %s\n", oledOK ? "found" : "not found");
}

void showMsg(const String& a, const String& b) {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0); display.println(a);
  display.setCursor(0, 18); display.println(b);
  display.display();
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
  display.setCursor(60, 0);
  display.print("SHUFFLE");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setTextWrap(true);
  display.setCursor(0, 18);
  display.print(cleanTitle(source.toStr()));
  display.display();
}

void buildShuffle(int n) {
  songCount = (n > MAX_SONGS) ? MAX_SONGS : n;
  for (int i = 0; i < songCount; i++) shuffleOrder[i] = i;
  for (int i = songCount - 1; i > 0; i--) {         // Fisher-Yates
    int j = esp_random() % (i + 1);
    int t = shuffleOrder[i]; shuffleOrder[i] = shuffleOrder[j]; shuffleOrder[j] = t;
  }
  shufflePos = 0;
}

// read the card currently on the reader; "" if none
String readCard() {
  for (int t = 0; t < 6; t++) {
    if (mfrc.PICC_IsNewCardPresent() && mfrc.PICC_ReadCardSerial()) {
      String uid = "";
      for (byte i = 0; i < mfrc.uid.size; i++) {
        if (mfrc.uid.uidByte[i] < 0x10) uid += "0";
        uid += String(mfrc.uid.uidByte[i], HEX);
      }
      uid.toUpperCase();
      mfrc.PICC_HaltA();
      mfrc.PCD_StopCrypto1();
      return uid;
    }
    delay(25);
  }
  return "";
}

bool loadAlbum(const String& uid) {
  albumPath = "/" + uid;
  source.setPath(albumPath.c_str());
  source.begin();                                   // re-index this album folder
  long n = source.size();
  if (n <= 0) { showMsg("No songs for card:", uid); Serial.println("empty album " + uid); return false; }
  currentAlbum = uid;
  buildShuffle((int)n);
  player.setIndex(shuffleOrder[0]);
  Serial.printf("Album %s : %ld songs\n", uid.c_str(), n);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== RFID Jukebox ===");

  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  initOLED();

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);              // shared SPI: SD (CS13) + RC522 (CS4)
  mfrc.PCD_Init();
  byte v = mfrc.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.printf("RC522: 0x%02X %s\n", v, (v == 0x91 || v == 0x92) ? "(OK)" : "(CHECK WIRING)");

  a2dpBuffer.resize(24 * 1024);
  out.begin(95);
  player.setDelayIfOutputFull(0);
  player.setVolume(0.70);   // headroom for safety
  player.begin(0, false);                           // mount SD + index, but don't play yet
  player.setAutoNext(false);                        // we drive shuffle manually

  a2dp.set_data_callback(get_data);
  a2dp.start(BT_DEVICE_NAME);

  showMsg("Tap a card", "to start music");
  Serial.println("Ready. Tap a card.");
}

void loop() {
  player.copy();                                    // feeds audio (or silence when idle)

  if (currentAlbum == "") {                         // waiting for the first card
    String uid = readCard();
    if (uid != "") loadAlbum(uid);
    return;
  }

  if (!player.isActive()) {                         // a song just finished
    String uid = readCard();                        // check the reader at the boundary
    if (uid != "" && uid != currentAlbum) {
      loadAlbum(uid);                               // different card -> switch album
    } else {
      shufflePos++;                                 // same/no card -> next shuffled song
      if (shufflePos >= songCount) buildShuffle(songCount);
      player.setIndex(shuffleOrder[shufflePos]);
    }
  }

  static uint32_t lastUI = 0;
  if (millis() - lastUI > 800) { lastUI = millis(); drawScreen(); }
}
