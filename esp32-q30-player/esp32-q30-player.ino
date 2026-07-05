/*
 * LoRa32 T3 (v1.6.1 / v2.1) MP3 player -> Soundcore Life Q30 (A2DP)
 * Joystick + OLED song-list version.
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
#define JOY_Y   34          
#define JOY_SW   4          
#define MAX_SONGS 150
#define LIMIT_PCT 100       
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

String songs[MAX_SONGS];
int songCount = 0;
int shuffleOrder[MAX_SONGS];
int shufflePos = 0;

enum { MODE_PLAY, MODE_LIST };
int uiMode = MODE_PLAY;
int cursor = 0;                 
int nowPlaying = -1;            
uint32_t lastMove = 0, lastActivity = 0;

int32_t get_data(uint8_t* d, int32_t n) {
  int32_t got = a2dpBuffer.readArray(d, n);
  int16_t* s = (int16_t*)d;
  int count = got / 2;
  for (int i = 0; i < count; i++) {
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
  for (int i = songCount - 1; i > 0; i--) {
    int j = esp_random() % (i + 1);
    int t = shuffleOrder[i]; shuffleOrder[i] = shuffleOrder[j]; shuffleOrder[j] = t;
  }
  shufflePos = 0;
}

void playSong(int idx) {
  if (idx < 0 || idx >= songCount) return;
  nowPlaying = idx;
  
  // 💡 핵심 수정 1: 다음 곡을 열기 전에 확실하게 이전 파일을 닫고 메모리를 해제! (파일 누수 완벽 방지)
  player.end(); 
  
  bool ok = player.setPath(songs[idx].c_str());
  Serial.printf("Playing[%d] %s -> %s\n", idx, songs[idx].c_str(), ok ? "OK" : "FAIL");
  
  // 💡 핵심 수정 2: 로딩 실패 시 0.5초 대기. (스킵 지옥으로 빠져 기기가 뻗는 것을 방어)
  if (!ok) {
    delay(500); 
  }
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
  
  // 💡 핵심 수정 3: 메모리가 숨 쉴 수 있게 버퍼를 12KB로 안정화
  a2dpBuffer.resize(12 * 1024);
  out.begin(95);
  player.setDelayIfOutputFull(0);
  player.setVolume(0.70);
  player.begin(0, false);
  player.setAutoNext(false);

  a2dp.set_data_callback(get_data);
  a2dp.start(BT_DEVICE_NAME);

  if (songCount > 0) { buildShuffle(); playSong(shuffleOrder[0]); }
  else if (oledOK) { display.clearDisplay(); display.setCursor(0,0); display.print("No songs on SD"); display.display(); }
}

void loop() {
  // 1. 오디오 버퍼 채우기
  player.copy();

  // 2. 노래 끝나면 다음 곡
  if (songCount > 0 && !player.isActive()) {
    playNextShuffle();
  }

  uint32_t now = millis();
  static uint32_t lastJoy = 0;
  static bool up = false, down = false;
  
  if (now - lastJoy >= 50) {
    lastJoy = now;
    int y = analogRead(JOY_Y);
    up = (y > 3300); down = (y < 800);
  }

  bool needsDrawList = false;
  bool needsDrawPlay = false;

  if ((up || down)) {
    if (uiMode == MODE_PLAY) { 
      uiMode = MODE_LIST; 
      cursor = (nowPlaying >= 0 ? nowPlaying : 0); 
      needsDrawList = true; 
      lastActivity = now; lastMove = now; 
    }
    else if (now - lastMove > 220) {
      if (up)   { cursor--; if (cursor < 0) cursor = 0; }
      if (down) { cursor++; if (cursor >= songCount) cursor = songCount - 1; }
      lastMove = now; lastActivity = now; 
      needsDrawList = true;
    }
  }

  static bool prevSw = HIGH;
  bool sw = digitalRead(JOY_SW);
  if (prevSw == HIGH && sw == LOW) {
    if (uiMode == MODE_LIST) { 
      playSong(cursor); 
      uiMode = MODE_PLAY; 
      needsDrawPlay = true;
    }
    lastActivity = now;
  }
  prevSw = sw;

  if (uiMode == MODE_LIST && now - lastActivity > 5000) { 
    uiMode = MODE_PLAY; 
    needsDrawPlay = true;
  }

  static int lastDrawnSong = -1;
  static bool lastConn = false;
  bool currConn = a2dp.is_connected();

  if (uiMode == MODE_PLAY) {
    if (nowPlaying != lastDrawnSong || currConn != lastConn) {
      needsDrawPlay = true;
      lastDrawnSong = nowPlaying;
      lastConn = currConn;
    }
  } else {
    lastDrawnSong = -1;
  }

  // 버퍼가 12KB로 줄었으니 채우는 횟수도 가볍게 조절
  if (needsDrawList || needsDrawPlay) {
    for (int i = 0; i < 12; i++) {
      if (player.copy() == 0) break; 
    }
    
    if (needsDrawList) drawList();
    if (needsDrawPlay) drawPlay();
  }
}
