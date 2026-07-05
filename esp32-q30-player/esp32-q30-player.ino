/*
 * LoRa32 T3 (v1.6.1 / v2.1) MP3 player -> Soundcore Life Q30 (A2DP)
 * Joystick + OLED song-list version.
 *
 *  - All .mp3 files live in the SD-card ROOT (no folders).
 *  - Plays SHUFFLED (no repeats until every song has played once), loops forever.
 *  - Push the joystick UP/DOWN to open a scrollable song list; press the stick (D)
 *    to jump to that song. After the picked song ends, shuffle resumes.
 *  - Volume from the headphones. Safety clamp on the audio output.
 *
 * Songs: MP3, 44.1kHz, stereo.
 *
 * Wiring (joystick):  Y -> IO34,  D(SW) -> IO4,  VCC -> 3.3V,  GND -> GND,  X -> (nc)
 * SD (built-in): SCK14 MISO2 MOSI15 CS13.  OLED (built-in): SDA21 SCL22.
 */

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "AudioTools.h"
#include "AudioTools/Communication/A2DPStream.h"
#include "AudioTools/Disk/AudioSourceSD.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"

// ---------- SETTINGS ----------
const char* BT_DEVICE_NAME = "Soundcore Life Q30";
#define SD_CS   13
#define SD_SCK  14
#define SD_MISO  2
#define SD_MOSI 15
#define LORA_CS 18
#define JOY_Y   34          // joystick Y axis (ADC)
#define JOY_SW   4          // joystick push button (to GND, internal pull-up)
#define MAX_SONGS 150
#define LIMIT_PCT 100       // audio safety ceiling (100 = transparent)
// ------------------------------
static const int16_t SAFE_LIMIT = (int16_t)(32767L * LIMIT_PCT / 100);

Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOK = false;

AudioSourceSD source("/", ".mp3", SD_CS);
MP3DecoderHelix decoder;
BufferRTOS<uint8_t> a2dpBuffer(0);
QueueStream<uint8_t> out(a2dpBuffer);
AudioPlayer player(source, out, decoder);
BluetoothA2DPSource a2dp;

// our own song list (full paths like "/Blue Bird.mp3")
String songs[MAX_SONGS];
int songCount = 0;
int shuffleOrder[MAX_SONGS];
int shufflePos = 0;

// UI state
enum { MODE_PLAY, MODE_LIST };
int uiMode = MODE_PLAY;
int cursor = 0;                 // highlighted song in list
int nowPlaying = -1;           // index (into songs[]) currently playing
uint32_t lastMove = 0, lastActivity = 0;

int32_t get_data(uint8_t* d, int32_t n) {
  int32_t got = a2dpBuffer.readArray(d, n);
  int16_t* s = (int16_t*)d;
  int count = got / 2;
  for (int i = 0; i < count; i++) {          // safety clamp
    if (s[i] >  SAFE_LIMIT) s[i] =  SAFE_LIMIT;
    else if (s[i] < -SAFE_LIMIT) s[i] = -SAFE_LIMIT;
  }
  return got;
}

String cleanTitle(const String& raw) {
  String s = raw;
  int slash = s.lastIndexOf('/');
  if (slash >= 0) s = s.substring(slash + 1);
  if (s.endsWith(".mp3") || s.endsWith(".MP3")) s = s.substring(0, s.length() - 4);
  return s;
}

void scanSongs() {
  songCount = 0;
  File root = SD.open("/");
  if (!root) { Serial.println("SD root open failed"); return; }
  File f = root.openNextFile();
  while (f && songCount < MAX_SONGS) {
    if (!f.isDirectory()) {
      String name = String(f.name());
      int sl = name.lastIndexOf('/'); if (sl >= 0) name = name.substring(sl + 1);
      String lower = name; lower.toLowerCase();
      if (lower.endsWith(".mp3")) songs[songCount++] = "/" + name;
    }
    f.close();
    f = root.openNextFile();
  }
  root.close();
  Serial.printf("Found %d songs\n", songCount);
}

void buildShuffle() {
  for (int i = 0; i < songCount; i++) shuffleOrder[i] = i;
  for (int i = songCount - 1; i > 0; i--) {          // Fisher-Yates (no repeats per cycle)
    int j = esp_random() % (i + 1);
    int t = shuffleOrder[i]; shuffleOrder[i] = shuffleOrder[j]; shuffleOrder[j] = t;
  }
  shufflePos = 0;
}

void playSong(int idx) {
  if (idx < 0 || idx >= songCount) return;
  nowPlaying = idx;
  player.setPath(songs[idx].c_str());
  Serial.println("Playing: " + songs[idx]);
}

void playNextShuffle() {
  shufflePos++;
  if (shufflePos >= songCount) buildShuffle();
  playSong(shuffleOrder[shufflePos]);
}

void drawPlay() {
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
  if (nowPlaying >= 0) display.print(cleanTitle(songs[nowPlaying]));
  display.display();
}

void drawList() {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  const int rows = 6;
  int start = cursor - rows / 2;
  if (start > songCount - rows) start = songCount - rows;
  if (start < 0) start = 0;
  for (int i = 0; i < rows && start + i < songCount; i++) {
    int idx = start + i;
    display.setCursor(0, i * 10);
    display.print(idx == cursor ? ">" : " ");
    String t = cleanTitle(songs[idx]);
    if (t.length() > 20) t = t.substring(0, 20);
    display.print(t);
  }
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== Joystick MP3 player ===");

  pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);
  pinMode(JOY_SW, INPUT_PULLUP);
  analogReadResolution(12);

  Wire.begin(21, 22); Wire.setClock(400000); Wire.setTimeOut(50);
  Wire.beginTransmission(0x3C);
  if (Wire.endTransmission() == 0) oledOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (!SD.begin(SD_CS, SPI)) Serial.println("SD mount failed");
  scanSongs();

  a2dpBuffer.resize(24 * 1024);
  out.begin(95);
  player.setDelayIfOutputFull(0);
  player.setVolume(0.70);            // headroom for safety
  player.begin(0, false);
  player.setAutoNext(false);

  a2dp.set_data_callback(get_data);
  a2dp.start(BT_DEVICE_NAME);

  if (songCount > 0) { buildShuffle(); playSong(shuffleOrder[0]); }
  else if (oledOK) { display.clearDisplay(); display.setCursor(0,0); display.print("No songs on SD"); display.display(); }
}

void loop() {
  player.copy();

  // song finished -> next shuffled track
  if (songCount > 0 && !player.isActive()) playNextShuffle();

  // ---- joystick ----
  int y = analogRead(JOY_Y);                 // 0..4095, center ~2048
  bool up = (y < 1200), down = (y > 2900);
  uint32_t now = millis();

  if ((up || down)) {
    if (uiMode == MODE_PLAY) { uiMode = MODE_LIST; cursor = (nowPlaying >= 0 ? nowPlaying : 0); drawList(); lastActivity = now; lastMove = now; }
    else if (now - lastMove > 160) {
      if (up)   { cursor--; if (cursor < 0) cursor = 0; }
      if (down) { cursor++; if (cursor >= songCount) cursor = songCount - 1; }
      lastMove = now; lastActivity = now; drawList();
    }
  }

  // button press = select current song (edge-detected)
  static bool prevSw = HIGH;
  bool sw = digitalRead(JOY_SW);
  if (prevSw == HIGH && sw == LOW) {
    if (uiMode == MODE_LIST) { playSong(cursor); uiMode = MODE_PLAY; drawPlay(); }
    lastActivity = now;
  }
  prevSw = sw;

  // auto-return from list to play view after 5s idle
  if (uiMode == MODE_LIST && now - lastActivity > 5000) { uiMode = MODE_PLAY; drawPlay(); }

  // refresh play view periodically
  static uint32_t lastUI = 0;
  if (uiMode == MODE_PLAY && now - lastUI > 800) { lastUI = now; drawPlay(); }
}
