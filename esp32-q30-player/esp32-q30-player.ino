/*
 * LoRa32 T3 MP3 player -> Soundcore Life Q30 (A2DP) + OLED 뮤직비디오
 *
 * ★ v3 — 곡 전환 재설계 (뽁뢱/전환실패 근본 해결)
 *   문제였던 것: 곡 전환을 loop(코어1)에서 g_songChanged로 요청 -> audioTask(코어0)가
 *     setPath+버퍼리셋. 이 '크로스코어 + 스트리밍 중 파이프라인 교체'가 A2DP 소스의
 *     fresh 상태를 깨서 전환 후 뽁뢱/정지가 났다. (첫 곡만 깨끗했던 이유 = 연결 직후 fresh)
 *   해결(라이브러리 정석): 전환을 오디오와 '같은 태스크(코어0)'에서 player.setIndex()로만
 *     처리하고, 라이브러리의 setSilenceOnInactive(true)로 전환 갭에 무음을 채워 A2DP
 *     스트림을 끊기지 않게 유지. loop은 영상/UI 전담 + 전환 '요청'만 넘긴다.
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

// ---------- VIDEO ----------
#define OLED_ADDR       0x3C
#define VIDEO_FPS       12
#define AUDIO_BPS       (44100UL * 2 * 2)
#define SYNC_OFFSET_MS  180
static const uint32_t BYTES_PER_FRAME   = AUDIO_BPS / VIDEO_FPS;
static const uint32_t SYNC_OFFSET_BYTES = (AUDIO_BPS * SYNC_OFFSET_MS) / 1000;
static const uint16_t FRAME_BYTES = 1024;
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
volatile int nowPlaying = -1;                 // audioTask -> loop (현재 곡 인덱스)
uint32_t lastMove = 0, lastActivity = 0;

// ---- video clock ----
volatile uint32_t g_bytesPlayed = 0;          // A2DP 소비 바이트 = 재생 위치
static portMUX_TYPE clkMux = portMUX_INITIALIZER_UNLOCKED;
File     videoFile;
uint32_t videoFrameCount = 0;
int32_t  lastVideoFrame = -1;

// ---- light 계측 (STAT 로그용) ----
volatile uint32_t g_underruns = 0;
volatile uint32_t g_getCalls  = 0;

// ---- 코어 간 공유 ----
SemaphoreHandle_t sdMutex;
volatile bool     g_audioActive    = false;   // audioTask -> loop
volatile int      g_reqIndex       = -1;      // loop -> audioTask: 이 곡으로 전환 요청
volatile int      g_videoPendingIdx = -1;     // audioTask -> loop: 이 곡 영상 열어라
volatile uint32_t g_videoPendingAt  = 0;

int32_t get_data(uint8_t* d, int32_t n) {
  g_getCalls++;
  int32_t got = a2dpBuffer.readArray(d, n);
  portENTER_CRITICAL(&clkMux);
  g_bytesPlayed += (uint32_t)got;             // 실제 소비 바이트만 카운트
  portEXIT_CRITICAL(&clkMux);
  if (got < n) { g_underruns++; memset(d + got, 0, (size_t)(n - got)); }
  int16_t* s = (int16_t*)d;
  int count = got / 2;
  for (int i = 0; i < count; i++) {
    if (s[i] >  SAFE_LIMIT) s[i] =  SAFE_LIMIT;
    else if (s[i] < -SAFE_LIMIT) s[i] = -SAFE_LIMIT;
  }
  return n;
}

String cleanTitle(const String& raw) {
  String s = raw;
  int slash = s.lastIndexOf('/');
  if (slash >= 0) s = s.substring(slash + 1);
  if (s.endsWith(".mp3") || s.endsWith(".MP3")) s = s.substring(0, s.length() - 4);
  return s;
}
String videoPathFor(const String& mp3) {
  String v = mp3;
  if (v.endsWith(".mp3") || v.endsWith(".MP3")) v = v.substring(0, v.length() - 4);
  v += ".bin";
  return v;
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

// ---- 영상 파일 열기 (코어1 전용) ----
void openVideoFor(int idx) {
  if (idx < 0 || idx >= songCount) return;
  videoFrameCount = 0;
  lastVideoFrame  = -1;
  String vp = videoPathFor(songs[idx]);
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  if (videoFile) videoFile.close();
  videoFile = SD.open(vp.c_str(), FILE_READ);
  bool ok = (videoFile && !videoFile.isDirectory());
  if (ok) videoFrameCount = videoFile.size() / FRAME_BYTES;
  xSemaphoreGive(sdMutex);
  if (ok) Serial.printf("video: %s (%u frames)\n", vp.c_str(), videoFrameCount);
  else { if (videoFile) videoFile.close(); Serial.printf("no video: %s\n", vp.c_str()); }
}

// audioTask가 곡을 바꾼 뒤 loop에 알림 (영상 열기 + 영상시계 리셋)
void announceSong(int idx) {
  nowPlaying = idx;
  portENTER_CRITICAL(&clkMux);
  g_bytesPlayed = 0;
  portEXIT_CRITICAL(&clkMux);
  g_videoPendingIdx = idx;                     // 코어1이 살짝 뒤에 영상 오픈(전환순간 코어1 한가하게)
  g_videoPendingAt  = millis() + 250;
  Serial.printf("Playing[%d] %s\n", idx, songs[idx].c_str());
}

// ============================================================
//  오디오 태스크 (코어0) — 디코딩 + 곡 전환을 '한 태스크에서' 처리
// ============================================================
void audioTask(void* pv) {
  static uint32_t songStart = 0;
  static bool     sawActive = false;
  for (;;) {
    // (1) 전환 요청 처리 (부팅 첫 곡 + 조이스틱 수동선택). 같은 태스크에서 setIndex.
    int req = g_reqIndex;
    if (req >= 0 && req < songCount) {
      g_reqIndex = -1;
      xSemaphoreTake(sdMutex, portMAX_DELAY);
      player.setIndex(req);                     // 라이브러리 정석 전환 (버퍼리셋/end 수동호출 안 함)
      xSemaphoreGive(sdMutex);
      songStart = millis(); sawActive = false;
      announceSong(req);
    }

    // (2) 디코딩 (버퍼 채우기)
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    player.copy();
    bool act = player.isActive();
    xSemaphoreGive(sdMutex);
    g_audioActive = act;
    if (act) sawActive = true;

    // (3) 진짜 곡 끝 -> 셔플 다음 곡 (연결됨 + 3초이상 재생됨 확인 -> 부팅/버퍼풀 오발 방지)
    if (sawActive && !act && g_reqIndex < 0 &&
        a2dp.is_connected() && millis() - songStart > 3000) {
      shufflePos++;
      if (shufflePos >= songCount) buildShuffle();
      int idx = shuffleOrder[shufflePos];
      xSemaphoreTake(sdMutex, portMAX_DELAY);
      player.setIndex(idx);
      xSemaphoreGive(sdMutex);
      songStart = millis(); sawActive = false;
      announceSong(idx);
    }

    vTaskDelay(1);
  }
}

void drawPlayTitle() {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(a2dp.is_connected() ? "Q30" : "...");
  display.setCursor(60, 0); display.print("SHUFFLE");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setTextWrap(true);
  display.setCursor(0, 18);
  int np = nowPlaying;
  if (np >= 0 && np < songCount) display.print(cleanTitle(songs[np]));
  display.display();
}

void renderVideo() {
  if (!oledOK) return;
  if (!videoFile || videoFrameCount == 0) {
    static int lastT = -2; static bool lastC = false;
    bool c = a2dp.is_connected();
    if (nowPlaying != lastT || c != lastC) { drawPlayTitle(); lastT = nowPlaying; lastC = c; }
    return;
  }
  uint32_t bp = g_bytesPlayed;
  uint32_t tf = (bp > SYNC_OFFSET_BYTES) ? (bp - SYNC_OFFSET_BYTES) / BYTES_PER_FRAME : 0;
  if (tf >= videoFrameCount) tf = videoFrameCount - 1;
  if ((int32_t)tf == lastVideoFrame) return;
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  videoFile.seek((uint32_t)tf * FRAME_BYTES);
  int r = videoFile.read(display.getBuffer(), FRAME_BYTES);
  xSemaphoreGive(sdMutex);
  if (r == (int)FRAME_BYTES) display.display();
  lastVideoFrame = (int32_t)tf;
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
  Serial.println("\n\n=== MP3 + OLED player (v3: in-task switching) ===");

  pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);
  pinMode(JOY_SW, INPUT_PULLUP);
  analogReadResolution(12);

  Wire.begin(21, 22); Wire.setClock(400000); Wire.setTimeOut(50);
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() == 0) oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (!SD.begin(SD_CS, SPI, 20000000)) Serial.println("SD mount failed");
  scanSongs();

  sdMutex = xSemaphoreCreateMutex();

  a2dpBuffer.resize(20 * 1024);
  out.begin(95);

  source.setTimeoutAutoNext(30000);            // ★ 연결 전(~20s) 버퍼풀로 곡 건너뛰는 오발 방지
  player.setDelayIfOutputFull(0);
  player.setVolume(0.70);
  player.setSilenceOnInactive(true);           // ★ 전환/비활성 갭에 무음 -> A2DP 스트림 유지
  player.setAutoNext(false);                   //   셔플 진행은 audioTask가 직접(진짜 EOF에서)
  player.begin(0, false);

  a2dp.set_data_callback(get_data);
  a2dp.start(BT_DEVICE_NAME);

  xTaskCreatePinnedToCore(audioTask, "audio", 8192, NULL, 2, NULL, 0);

  if (songCount > 0) { buildShuffle(); g_reqIndex = shuffleOrder[0]; }  // 첫 곡 요청
  else if (oledOK) { display.clearDisplay(); display.setCursor(0,0);
                     display.print("No songs on SD"); display.display(); }
}

void loop() {
  // 코어1: 영상 + UI. 오디오/전환은 audioTask(코어0)가 전담.

  // 2초마다 상태 로그
  {
    static uint32_t statT = 0;
    uint32_t nowS = millis();
    if (nowS - statT > 2000) {
      statT = nowS;
      uint32_t ur = g_underruns, gc = g_getCalls; g_underruns = 0; g_getCalls = 0;
      Serial.printf("[STAT] UR=%u calls=%u conn=%d act=%d np=%d buf=%u pos=%.1fs\n",
                    ur, gc, (int)a2dp.is_connected(), (int)g_audioActive, nowPlaying,
                    (uint32_t)a2dpBuffer.available(), g_bytesPlayed / (float)AUDIO_BPS);
    }
  }

  // audioTask가 곡을 바꿨으면 영상 오픈 (전환순간 지나 코어1이 한가할 때)
  if (g_videoPendingIdx >= 0 && (int32_t)(millis() - g_videoPendingAt) >= 0) {
    int v = g_videoPendingIdx; g_videoPendingIdx = -1;
    openVideoFor(v);
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
  if (up || down) {
    if (uiMode == MODE_PLAY) {
      uiMode = MODE_LIST;
      cursor = (nowPlaying >= 0 ? nowPlaying : 0);
      needsDrawList = true; lastActivity = now; lastMove = now;
    } else if (now - lastMove > 220) {
      if (up)   { cursor--; if (cursor < 0) cursor = 0; }
      if (down) { cursor++; if (cursor >= songCount) cursor = songCount - 1; }
      lastMove = now; lastActivity = now; needsDrawList = true;
    }
  }

  static bool prevSw = HIGH;
  static uint32_t lastSel = 0;
  bool sw = digitalRead(JOY_SW);
  if (prevSw == HIGH && sw == LOW) {
    if (uiMode == MODE_LIST && cursor != nowPlaying && now - lastSel > 700) {
      g_reqIndex = cursor;                       // ★ audioTask에 전환 요청 (디바운스)
      lastSel = now;
      uiMode = MODE_PLAY;
    } else if (uiMode == MODE_LIST) {
      uiMode = MODE_PLAY;
    }
    lastActivity = now;
  }
  prevSw = sw;

  if (uiMode == MODE_LIST && now - lastActivity > 5000) {
    uiMode = MODE_PLAY; lastVideoFrame = -1;
  }

  if (uiMode == MODE_LIST) { if (needsDrawList) drawList(); }
  else                     { renderVideo(); }

  vTaskDelay(1);
}
