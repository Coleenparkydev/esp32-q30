/*
 * LoRa32 T3 MP3 player -> Soundcore Life Q30 (A2DP)
 * + 동기화된 OLED 뮤직비디오 재생 (SSD1306 128x64, 1비트 .bin)
 *
 * ★ v5 — 소리 나던 v2(수제 BluetoothA2DPSource + get_data pull) 로 복귀.
 *   v4(A2DPStream push)는 이 라이브러리 버전에서 실제로 소리를 못 냈다.
 *   여기에 소리를 깨뜨릴 위험 없는 안전한 팝핑 완화만 얹음:
 *     FIX6  get_data: 언더런이어도 항상 n 반환(모자란 부분 무음 0으로 채움)
 *           -> 짧게 반환하면 SBC 인코더가 거친 잡음("뽂뽂")을 냄. 무음채움=안 들림.
 *     FIX9  SD.begin 20MHz -> 디코더가 버퍼를 깊게 채워 언더런↓ + 영상 fps↑
 *     FIX5  곡 전환 직후 버퍼 프리필 -> 2번째 곡도 쿠션 갖고 시작
 *     FIX12 전환은 setPath만. player.end()는 get_data 콜백과 레이스->데드락이라 안 씀.
 *
 * 영상 원리:
 *   - PC 인코더의 songXX.bin (프레임당 1024B, SSD1306 버퍼 포맷)
 *   - get_data(A2DP 콜백)가 소비한 '실제' 바이트 = 재생 위치 (마스터 시계)
 *   - 지금 프레임 = (재생위치 - 헤드폰지연) / 프레임당바이트
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
volatile uint32_t g_bytesPlayed = 0;   // A2DP 소비 '실제' 바이트 = 재생 위치 (마스터 시계)
static portMUX_TYPE clkMux = portMUX_INITIALIZER_UNLOCKED;
File     videoFile;
uint32_t videoFrameCount = 0;
int32_t  lastVideoFrame = -1;

// ---- 코어 간 공유 ----
SemaphoreHandle_t sdMutex;
volatile bool     g_songChanged = false;
volatile int      g_requestSong = -1;
volatile bool     g_audioActive = false;
volatile bool     g_switching   = false;   // 전환 중엔 get_data 를 무음으로 -> end/reset 레이스 차단

// ============================================================
//  A2DP 데이터 콜백 (BT 스택이 PCM 을 pull) — 이 pull 모델이 v2에서 소리를 냈다.
// ============================================================
int32_t get_data(uint8_t* d, int32_t n) {
  // ★ 전환 중이면 버퍼를 절대 건드리지 않고 무음만 반환 -> audioTask 의 end()/reset() 과
  //   이 콜백이 동시에 a2dpBuffer 를 만지는 레이스(=데드락/오염)를 원천 차단.
  if (g_switching) { memset(d, 0, (size_t)n); return n; }
  int32_t got = a2dpBuffer.readArray(d, n);
  portENTER_CRITICAL(&clkMux);
  g_bytesPlayed += (uint32_t)got;       // 마스터 시계 = 실제로 흘려보낸 바이트만
  portEXIT_CRITICAL(&clkMux);

  // ★ FIX6: 언더런(got<n)이어도 나머지를 무음(0)으로 채우고 '항상 n' 반환.
  //   짧게 반환하면 SBC 인코더가 거친 잡음("뽂뽂")을 낸다. 무음채움=안 들리는 미세공백.
  if (got < n) memset(d + got, 0, (size_t)(n - got));

  // 클리핑 방지 클램프 (실제 채워진 got 바이트에만)
  int16_t* s = (int16_t*)d;
  int count = got / 2;
  for (int i = 0; i < count; i++) {
    if (s[i] >  SAFE_LIMIT) s[i] =  SAFE_LIMIT;
    else if (s[i] < -SAFE_LIMIT) s[i] = -SAFE_LIMIT;
  }
  return n;                              // ★ 항상 요청한 만큼 반환
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
        // ★ 전환 프로토콜: get_data 를 먼저 무음으로 세운 뒤(레이스 차단) 깨끗이 정리.
        //   이렇게 하면 end()/reset() 를 안전하게 부를 수 있어 데드락 없이도
        //   이전 곡 디코더/스트림/버퍼가 완전히 리셋됨 -> "틀수록 이상해짐" 누적 오염 해결.
        // 가드는 '버퍼에 안 쓰는' 정리 구간(end/reset/setPath)에만 건다.
        g_switching = true;                    // get_data 무음화 -> 아래 reset 과의 레이스 차단
        vTaskDelay(pdMS_TO_TICKS(8));          // 진행 중이던 get_data 콜이 빠져나갈 시간
        xSemaphoreTake(sdMutex, portMAX_DELAY);
        player.end();                          // 이전 곡 스트림/디코더 정리 (누수/오염 방지)
        a2dpBuffer.reset();                    // 이전 곡 PCM flush (get_data 가드 상태라 안전)
        portENTER_CRITICAL(&clkMux);
        g_bytesPlayed = 0;                     // 시계=0 을 새 곡에 정렬 (영상 리셋)
        portEXIT_CRITICAL(&clkMux);
        bool ok = player.setPath(songs[idx].c_str());
        xSemaphoreGive(sdMutex);
        g_switching = false;                   // ★ copy 하기 '전에' 드레인부터 재개!
        // 프리필 안 함: 가드 켜진 채로 copy() 하면 get_data 가 버퍼를 안 비워
        //   a2dpBuffer 쓰기가 영원히 블로킹 -> audioTask 데드락(=v6 전부 멈춘 원인).
        //   이제 버퍼는 메인루프 copy 가 채우고 get_data 가 동시에 비우므로 안 막힘.
        //   시작 갭은 FIX6(언더런 무음채움)이 조용히 덮는다.
        Serial.printf("Playing[%d] %s -> %s\n", idx, songs[idx].c_str(), ok ? "OK" : "FAIL");
        if (!ok) vTaskDelay(pdMS_TO_TICKS(500));
      }
    }

    xSemaphoreTake(sdMutex, portMAX_DELAY);    // SD 접근 보호
    size_t n = player.copy();
    g_audioActive = player.isActive();
    xSemaphoreGive(sdMutex);

    if (n == 0) vTaskDelay(pdMS_TO_TICKS(2));  // 버퍼 가득 -> 양보
    else        vTaskDelay(1);                 // 1틱 양보 -> 코어0 idle/WDT 굶기 방지
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

  // ★ 오디오 우선(FIX-SD): 디코드 버퍼가 얕으면 이번 영상 프레임을 스킵하고
  //   SD 를 디코더에 양보 -> 언더런(0.7s 렉) 제거. 영상 프레임 드롭 << 오디오 끊김.
  //   곡 전환 직후 빈 버퍼 재충전도 이걸로 빨라져 전환 렉도 완화.
  if (a2dpBuffer.available() < 8 * 1024) return;

  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  videoFile.seek((uint32_t)tf * FRAME_BYTES);
  int r = videoFile.read(display.getBuffer(), FRAME_BYTES);
  xSemaphoreGive(sdMutex);

  if (r == (int)FRAME_BYTES) display.display();      // I2C blit (SD 잠금 밖)
  lastVideoFrame = (int32_t)tf;

  static uint32_t dbgT = 0, blits = 0;
  blits++;
  uint32_t nowMs = millis();
  if (nowMs - dbgT > 2000) {
    Serial.printf("fps~%.1f  frame=%d/%u  pos=%.1fs  conn=%d  buf=%uB  heap=%u\n",
                  blits / 2.0f, lastVideoFrame, videoFrameCount,
                  bp / (float)AUDIO_BPS, (int)a2dp.is_connected(),
                  (unsigned)a2dpBuffer.available(), ESP.getFreeHeap());
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
  Serial.println("\n\n=== MP3 + OLED video player (v8: decode->core1, audio-priority SD) ===");

  pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);
  pinMode(JOY_SW, INPUT_PULLUP);
  analogReadResolution(12);

  Wire.begin(21, 22); Wire.setClock(400000); Wire.setTimeOut(50);
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() == 0) oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  // ★ FIX9: SD 20MHz (기본 4MHz면 읽기가 5배 느려 버퍼가 안 참 -> 언더런/팝 + 영상 fps↓)
  //   "SD mount failed" 뜨면 카드가 20MHz 못 버티는 것 -> 16000000 으로 낮출 것.
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (!SD.begin(SD_CS, SPI, 20000000)) Serial.println("SD mount failed");
  scanSongs();

  sdMutex = xSemaphoreCreateMutex();

  a2dpBuffer.resize(16 * 1024);               // FIX4: 12->16KB 쿠션 (힙 여유 봐가며)
  out.begin(95);
  player.setDelayIfOutputFull(0);
  player.setVolume(0.70);
  player.begin(0, false);
  player.setAutoNext(false);

  a2dp.set_data_callback(get_data);
  a2dp.start(BT_DEVICE_NAME);

  // ★ FIX-CORE: 디코드 태스크를 core 1 로. BT 스택(컨트롤러+Bluedroid+A2DP SBC 인코딩)은
  //   core 0 에 고정돼 돌기 때문에, 디코드를 core 0 에 두면 BT 와 CPU 를 다퉈 굶는다
  //   -> 주기적 버퍼 언더런(0.7s 렉). core 1 로 옮겨 BT 에 core 0 을 통째로 내준다.
  //   (pschatzmann discussion #1930: "무선 스택 쓰면 audio task 는 core 1 을 써라")
  //   영상(loop)도 core 1 이지만 audioTask 우선순위(2)가 loop(1)보다 높고, 버퍼 얕으면
  //   renderVideo 가 SD 를 양보하므로 오디오가 항상 우선된다.
  xTaskCreatePinnedToCore(audioTask, "audio", 8192, NULL, 2, NULL, 1);

  if (songCount > 0) { buildShuffle(); requestSong(shuffleOrder[0]); }
  else if (oledOK) {
    display.clearDisplay(); display.setCursor(0,0);
    display.print("No songs on SD"); display.display();
  }
}

void loop() {
  // 오디오는 코어0 태스크가 담당. 여기(코어1)는 영상 + UI 전담.

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
    lastVideoFrame = -1;
  }

  if (uiMode == MODE_LIST) {
    if (needsDrawList) drawList();
  } else {
    renderVideo();
  }

  vTaskDelay(1);
}
