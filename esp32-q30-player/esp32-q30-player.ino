/*
 * LoRa32 T3 (v1.6.1 / v2.1) MP3 player -> Soundcore Life Q30 (A2DP)
 * Joystick + OLED song-list version.
 *
 * - All .mp3 files live in the SD-card ROOT (no folders).
 * - Plays SHUFFLED (no repeats until every song has played once), loops forever.
 * - Push the joystick UP/DOWN to open a scrollable song list; press the stick (D)
 * to jump to that song. After the picked song ends, shuffle resumes.
 * - Volume from the headphones. Safety clamp on the audio output.
 *
 * Songs: MP3, 44.1kHz, stereo.
 *
 * Wiring (joystick):  Y -> IO34,  D(SW) -> IO4,  VCC -> 3.3V,  GND -> GND,  X -> (nc)
 * SD (built-in): SCK14 MISO2 MOSI15 CS13. OLED (built-in): SDA21 SCL22.
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
int nowPlaying = -1;            // index (into songs[]) currently playing
uint32_t lastMove = 0, lastActivity = 0;

// --- 멀티태스킹(FreeRTOS)을 위한 전역 변수 ---
volatile bool isChangingSong = false;
TaskHandle_t audioTask;

// --- 백그라운드 오디오 재생 태스크 ---
void audioLoop(void *param) {
  for(;;) {
    if (!isChangingSong && player.isActive()) {
      player.copy();
    }
    vTaskDelay(1); // FreeRTOS 워치독이 뻗지 않게 1ms 양보
  }
}

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
  for (int i = songCount - 1; i > 0; i--) {          // Fisher-Yates
    int j = esp_random() % (i + 1);
    int t = shuffleOrder[i]; shuffleOrder[i] = shuffleOrder[j]; shuffleOrder[j] = t;
  }
  shufflePos = 0;
}

void playSong(int idx) {
  if (idx < 0 || idx >= songCount) return;
  isChangingSong = true; // SD카드 충돌 방지를 위해 오디오 태스크 일시정지
  nowPlaying = idx;
  bool ok = player.setPath(songs[idx].c_str());
  Serial.printf("Playing[%d] %s -> %s\n", idx, songs[idx].c_str(), ok ? "OK" : "FAIL");
  isChangingSong = false; // 재생 시작 후 오디오 태스크 재개
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

  // --- 오디오 전용 태스크 시작 ---
  xTaskCreatePinnedToCore(
    audioLoop,     // 실행할 함수
    "AudioTask",   // 태스크 이름
    10240,         // 스택 크기
    NULL,          // 파라미터
    2,             // 우선순위 (UI 루프보다 높게)
    &audioTask,    // 태스크 핸들
    1              // 코어 1 할당
  );
}

void loop() {
  // player.copy(); <-- 삭제됨! (audioLoop 태스크가 백그라운드에서 처리함)

  // 노래가 끝났을 때 다음 곡 재생 (충돌 방지를 위해 isChangingSong 체크)
  if (songCount > 0 && !isChangingSong && !player.isActive()) {
    playNextShuffle();
  }

  // ---- joystick: 50ms 마다 조작 상태 읽기 ----
  uint32_t now = millis();
  static uint32_t lastJoy = 0;
  static bool up = false, down = false;
  if (now - lastJoy >= 50) {
    lastJoy = now;
    int y = analogRead(JOY_Y);               // single read; wide dead-zone filters noise
    up = (y > 3300); down = (y < 800);       // Y inverted (push up = up)
  }

  if ((up || down)) {
    if (uiMode == MODE_PLAY) { 
      uiMode = MODE_LIST; 
      cursor = (nowPlaying >= 0 ? nowPlaying : 0); 
      drawList(); 
      lastActivity = now; 
      lastMove = now; 
    }
    else if (now - lastMove > 220) {
      if (up)   { cursor--; if (cursor < 0) cursor = 0; }
      if (down) { cursor++; if (cursor >= songCount) cursor = songCount - 1; }
      lastMove = now; lastActivity = now; drawList();
    }
  }

  // 조이스틱 버튼 누름 (곡 선택)
  static bool prevSw = HIGH;
  bool sw = digitalRead(JOY_SW);
  if (prevSw == HIGH && sw == LOW) {
    if (uiMode == MODE_LIST) { 
      playSong(cursor); 
      uiMode = MODE_PLAY; 
    }
    lastActivity = now;
  }
  prevSw = sw;

  // 5초 동안 입력 없으면 리스트 뷰에서 재생 뷰로 복귀
  if (uiMode == MODE_LIST && now - lastActivity > 5000) { 
    uiMode = MODE_PLAY; 
  }

  // ---- 스마트 UI 렌더링 (800ms 강제 갱신 제거) ----
  static int lastDrawnSong = -1;
  static bool lastConn = false;
  bool currConn = a2dp.is_connected();

  if (uiMode == MODE_PLAY) {
    // 곡이 바뀌었거나 블루투스 연결 상태가 바뀌었을 때만 화면 갱신
    if (nowPlaying != lastDrawnSong || currConn != lastConn) {
      drawPlay();
      lastDrawnSong = nowPlaying;
      lastConn = currConn;
    }
  } else {
    lastDrawnSong = -1; // 리스트 모드일 때 초기화해서 재생 모드 복귀 시 즉시 갱신 유도
  }
}
