/*
 * LoRa32 T3 MP3 player -> Soundcore Life Q30 (A2DP)
 * + 동기화된 OLED 뮤직비디오 재생 (SSD1306 128x64, 1비트 .bin)
 *
 * ★ v2 변경점 (fps~1 문제 해결)
 *   문제: player.copy()(MP3 디코딩)가 loop()을 통째로 잡아먹어서
 *         loop()이 초당 1~2회만 돌았음 -> 영상이 초당 1프레임밖에 못 그림.
 *   해결: 오디오 디코딩을 전용 FreeRTOS 태스크(코어0)로 분리.
 *         loop()(코어1)은 영상 + UI만 전담 -> 12fps 여유.
 *         BufferRTOS 가 스레드 세이프하므로 두 코어가 안전하게 통신.
 *         SD 는 두 코어가 공유하므로 뮤텍스로 보호.
 *
 * 영상 원리:
 *   - PC 인코더의 songXX.bin (프레임당 1024B, SSD1306 버퍼 포맷)
 *   - get_data(A2DP 콜백)가 소비한 바이트 = 재생 위치 (마스터 시계)
 *   - 지금 프레임 = (재생위치 - 헤드폰지연) / 프레임당바이트
 *   - display.getBuffer()에 직접 읽어넣고 display(). 밀리면 프레임 드롭.
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
#define VIDEO_FPS       12                      // .bin 인코딩 fps와 반드시 일치
#define AUDIO_BPS       (44100UL * 2 * 2)       // 176400 B/s
// 헤드폰 지연 보정(ms): 영상이 소리보다 빠르면 값 ↑, 늦으면 ↓
#define SYNC_OFFSET_MS  180
static const uint32_t BYTES_PER_FRAME   = AUDIO_BPS / VIDEO_FPS;                // 14700
static const uint32_t SYNC_OFFSET_BYTES = (AUDIO_BPS * SYNC_OFFSET_MS) / 1000;  // ~31752
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
int nowPlaying = -1;
uint32_t lastMove = 0, lastActivity = 0;

// ---- video state ----
volatile uint32_t g_bytesPlayed = 0;   // A2DP 소비 바이트 = 재생 위치 (마스터 시계)
// ★ FIX3: 마스터 시계 보호. get_data(+=)는 read-modify-write라, 다른 코어가
//   그 사이에 0을 쓰면 리셋이 씹혀서(clobber) 곡 전환 후 영상이 끝프레임에
//   붙어버림. 증분과 리셋을 이 스핀락으로 상호배제 -> 리셋이 항상 먹음.
static portMUX_TYPE clkMux = portMUX_INITIALIZER_UNLOCKED;
File     videoFile;
uint32_t videoFrameCount = 0;
int32_t  lastVideoFrame = -1;

// ---- 코어 간 공유 ----
SemaphoreHandle_t sdMutex;                 // SD 를 두 코어가 쓰므로 보호 (필수)
volatile bool     g_songChanged = false;   // loop -> 오디오 태스크 요청
volatile int      g_requestSong = -1;
volatile bool     g_audioActive = false;   // 오디오 태스크 -> loop 상태

int32_t get_data(uint8_t* d, int32_t n) {
  int32_t got = a2dpBuffer.readArray(d, n);
  portENTER_CRITICAL(&clkMux);
  g_bytesPlayed += (uint32_t)got;      // ★ 마스터 시계 (RMW -> 스핀락 보호). 실제
  portEXIT_CRITICAL(&clkMux);          //    디코딩된 바이트만 세야 영상 싱크 유지.
  // ★ FIX6: 언더런(got < n)이면 나머지를 무음(0)으로 채우고 꽉 찬 프레임을 반환.
  //   부분 전송/미초기화 버퍼가 그대로 나가면 SBC 인코더가 잡음('뽁뽁')을 냄.
  //   무음으로 패딩하면 최악의 경우도 깔끔한 무음이 됨.
  if (got < n) memset(d + got, 0, n - got);
  int16_t* s = (int16_t*)d;
  int count = got / 2;                 // 클램프는 실제 샘플만 (패딩된 무음은 이미 0)
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

// ---- 영상 파일 열기 (코어1에서만 호출) ----
void openVideoFor(int idx) {
  if (videoFile) videoFile.close();
  videoFrameCount = 0;
  lastVideoFrame  = -1;
  String vp = videoPathFor(songs[idx]);
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  videoFile = SD.open(vp.c_str(), FILE_READ);
  bool ok = (videoFile && !videoFile.isDirectory());
  if (ok) videoFrameCount = videoFile.size() / FRAME_BYTES;
  xSemaphoreGive(sdMutex);
  if (ok) {
    Serial.printf("video: %s (%u frames)\n", vp.c_str(), videoFrameCount);
  } else {
    if (videoFile) videoFile.close();
    Serial.printf("no video: %s (title only)\n", vp.c_str());
  }
}

// ---- 곡 요청 (loop에서 호출 -> 오디오 태스크가 실제 전환) ----
void requestSong(int idx) {
  if (idx < 0 || idx >= songCount) return;
  nowPlaying = idx;
  // g_bytesPlayed 리셋은 오디오 태스크가 버퍼 flush와 함께 수행 (FIX1) ->
  // 이전 곡 PCM(~12KB) 이 새 영상 시계에 섞이지 않게 정렬.
  g_requestSong = idx;
  g_songChanged = true;
  openVideoFor(idx);            // 영상 파일은 코어1이 직접 염
}

// ============================================================
//  오디오 태스크 (코어0 전용) - MP3 디코딩 담당
// ============================================================
void audioTask(void* pv) {
  for (;;) {
    if (g_songChanged) {                       // 곡 전환 요청 처리
      g_songChanged = false;
      int idx = g_requestSong;
      if (idx >= 0 && idx < songCount) {
        xSemaphoreTake(sdMutex, portMAX_DELAY);
        player.end();
        a2dpBuffer.reset();                    // ★ FIX1: 이전 곡 PCM 비우기
        portENTER_CRITICAL(&clkMux);
        g_bytesPlayed = 0;                     //   시계=0 을 새 곡 첫 바이트에 정렬
        portEXIT_CRITICAL(&clkMux);
        bool ok = player.setPath(songs[idx].c_str());
        // ★ FIX5: 프리필. 첫 곡은 BT 연결 전 버퍼가 저절로 꽉 차서 쿠션이 있지만,
        //   2번째 곡부터는 reset()으로 0이 된 버퍼를 get_data가 곧바로 빼가서
        //   바닥에서 맴돌다 SD 지연마다 언더런 -> 주기적 '뽁뽁'. 그래서 정상재생
        //   진입 전에 여기서 버퍼를 미리 채워 쿠션을 만든다. 디코딩(SD+Helix)이
        //   재생속도(176KB/s)보다 훨씬 빨라서 수십 ms면 채워짐. sdMutex를 계속
        //   쥐고 있어 이 동안 영상은 프레임 몇 개 드롭되지만 전환 순간이라 무해.
        if (ok) {
          size_t target = a2dpBuffer.size() * 3 / 4;   // 75%까지 쿠션 확보
          uint32_t t0 = millis();
          while (a2dpBuffer.available() < target && millis() - t0 < 800) {
            if (player.copy() == 0) break;             // 더 채울 게 없으면 탈출
          }
        }
        xSemaphoreGive(sdMutex);
        Serial.printf("Playing[%d] %s -> %s (prefill %u/%u)\n", idx, songs[idx].c_str(),
                      ok ? "OK" : "FAIL", (unsigned)a2dpBuffer.available(),
                      (unsigned)a2dpBuffer.size());
        if (!ok) vTaskDelay(pdMS_TO_TICKS(500));
      }
    }

    xSemaphoreTake(sdMutex, portMAX_DELAY);    // SD 접근 보호
    size_t n = player.copy();
    g_audioActive = player.isActive();
    xSemaphoreGive(sdMutex);

    if (n == 0) vTaskDelay(pdMS_TO_TICKS(2));  // 버퍼 가득 -> 양보
    else        vTaskDelay(1);                 // ★ FIX2: 1틱 양보 -> 코어0 idle/WDT 굶기 방지
  }
}

// ---- 영상 없는 곡용 제목 화면 ----
void drawPlayTitle() {
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

// ---- 영상 프레임 렌더 (코어1) ----
void renderVideo() {
  if (!oledOK) return;

  if (!videoFile || videoFrameCount == 0) {          // 영상 없는 곡 -> 제목만
    static int lastT = -2; static bool lastC = false;
    bool c = a2dp.is_connected();
    if (nowPlaying != lastT || c != lastC) {
      drawPlayTitle(); lastT = nowPlaying; lastC = c;
    }
    return;
  }

  uint32_t bp = g_bytesPlayed;
  uint32_t tf = (bp > SYNC_OFFSET_BYTES) ? (bp - SYNC_OFFSET_BYTES) / BYTES_PER_FRAME : 0;
  if (tf >= videoFrameCount) tf = videoFrameCount - 1;
  if ((int32_t)tf == lastVideoFrame) return;         // 아직 같은 프레임

  // SD 읽기 (오디오 태스크와 공유 -> 뮤텍스). 못 잡으면 이번 프레임 스킵.
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  videoFile.seek((uint32_t)tf * FRAME_BYTES);
  int r = videoFile.read(display.getBuffer(), FRAME_BYTES);
  xSemaphoreGive(sdMutex);

  if (r == (int)FRAME_BYTES) display.display();      // I2C blit (SD 잠금 밖)
  lastVideoFrame = (int32_t)tf;

  // 디버그: 2초마다 실제 fps
  static uint32_t dbgT = 0, blits = 0;
  blits++;
  uint32_t nowMs = millis();
  if (nowMs - dbgT > 2000) {
    Serial.printf("fps~%.1f  frame=%d/%u  pos=%.1fs  heap=%u\n",
                  blits / 2.0f, lastVideoFrame, videoFrameCount,
                  bp / (float)AUDIO_BPS, ESP.getFreeHeap());
    blits = 0; dbgT = nowMs;
  }
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
  Serial.println("\n\n=== MP3 + OLED video player (v2: dual-core) ===");

  pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);
  pinMode(JOY_SW, INPUT_PULLUP);
  analogReadResolution(12);

  Wire.begin(21, 22); Wire.setClock(400000); Wire.setTimeOut(50);
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() == 0) oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (!SD.begin(SD_CS, SPI)) Serial.println("SD mount failed");
  scanSongs();

  sdMutex = xSemaphoreCreateMutex();          // ★ SD 공유 보호

  a2dpBuffer.resize(32 * 1024);        // ★ FIX4: 12KB(68ms)->32KB(~185ms) 쿠션.
                                       //    SD seek 지연(10~20ms) 흡수해 언더런 방지.
  out.begin(95);
  player.setDelayIfOutputFull(0);
  player.setVolume(0.70);
  player.begin(0, false);
  player.setAutoNext(false);

  a2dp.set_data_callback(get_data);
  a2dp.start(BT_DEVICE_NAME);

  // ★ 오디오 디코딩을 코어0 전용 태스크로 (loop=코어1은 영상 전담)
  xTaskCreatePinnedToCore(audioTask, "audio", 8192, NULL, 2, NULL, 0);

  if (songCount > 0) { buildShuffle(); requestSong(shuffleOrder[0]); }
  else if (oledOK) {
    display.clearDisplay(); display.setCursor(0,0);
    display.print("No songs on SD"); display.display();
  }
}

void loop() {
  // 오디오는 코어0 태스크가 담당. 여기(코어1)는 영상 + UI 전담.

  // 곡 끝나면 다음 곡 (전환 직후 3초는 무시 -> 스킵 폭주 방지)
  static uint32_t songStart = 0;
  if (g_songChanged) songStart = millis();
  if (songCount > 0 && !g_audioActive && millis() - songStart > 3000) {
    songStart = millis();
    shufflePos++;
    if (shufflePos >= songCount) buildShuffle();
    requestSong(shuffleOrder[shufflePos]);
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

  if ((up || down)) {
    if (uiMode == MODE_PLAY) {
      uiMode = MODE_LIST;
      cursor = (nowPlaying >= 0 ? nowPlaying : 0);
      needsDrawList = true;
      lastActivity = now; lastMove = now;
    } else if (now - lastMove > 220) {
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
      requestSong(cursor);
      uiMode = MODE_PLAY;
    }
    lastActivity = now;
  }
  prevSw = sw;

  if (uiMode == MODE_LIST && now - lastActivity > 5000) {
    uiMode = MODE_PLAY;
    lastVideoFrame = -1;                 // 복귀 시 강제 재그리기
  }

  if (uiMode == MODE_LIST) {
    if (needsDrawList) drawList();
  } else {
    renderVideo();                       // 오디오 위치에 맞춰 프레임 (드롭 허용)
  }

  vTaskDelay(1);                         // 코어1 워치독 먹이기 + 태스크 양보
}
