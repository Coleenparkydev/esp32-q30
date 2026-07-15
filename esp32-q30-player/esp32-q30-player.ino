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

// ---- 1단계 계측 (순수 관찰용, 동작 변화 없음) ----
//   콜백에서 lock 없이 갱신 -> 카운터라 드문 레이스는 무시 가능(콜백은 빨라야 함).
//   loop()에서 2초마다 스냅샷+리셋해서 출력.
volatile uint32_t g_underruns = 0;          // got<n (버퍼 바닥) 발생 횟수
volatile uint32_t g_underrunB = 0;          // 무음(0)으로 채운 총 바이트
volatile uint32_t g_minBuf    = 0xFFFFFFFF; // 콜백 진입시 버퍼 최저 잔량(B)
volatile uint32_t g_getCalls  = 0;          // get_data 호출 횟수(참고용)

// ---- 코어 간 공유 ----
SemaphoreHandle_t sdMutex;                 // SD 를 두 코어가 쓰므로 보호 (필수)
volatile bool     g_songChanged = false;   // loop -> 오디오 태스크 요청
volatile int      g_requestSong = -1;
volatile bool     g_audioActive = false;   // 오디오 태스크 -> loop 상태

int32_t get_data(uint8_t* d, int32_t n) {
  uint32_t before = (uint32_t)a2dpBuffer.available();  // [계측] 읽기 전 버퍼 잔량
  if (before < g_minBuf) g_minBuf = before;            // [계측] 창구간 최저 수위
  g_getCalls++;
  int32_t got = a2dpBuffer.readArray(d, n);
  portENTER_CRITICAL(&clkMux);
  g_bytesPlayed += (uint32_t)got;      // ★ 마스터 시계: 실제 오디오 바이트만 카운트
  portEXIT_CRITICAL(&clkMux);          //    (무음 패딩은 제외해야 영상 싱크 유지)
  // ★ FIX6(복원): 라이브러리 저자 지침 - 콜백은 '항상' 요청량(n)을 채워 리턴.
  //   언더런(got < n)에 짧게 리턴하면 SBC 인코더가 거친 잡음('뽁뽁')을 냄.
  //   모자란 만큼 무음(0)으로 패딩 -> 최악이라도 안 들리는 짧은 무음이 됨.
  if (got < n) {
    g_underruns++;                          // [계측] 이번이 뽁뽁 후보
    g_underrunB += (uint32_t)(n - got);     // [계측] 무음으로 때운 양
    memset(d + got, 0, (size_t)(n - got));
  }
  int16_t* s = (int16_t*)d;
  int count = got / 2;                 // 클램프는 실제 샘플만 (패딩된 무음은 이미 0)
  for (int i = 0; i < count; i++) {
    if (s[i] >  SAFE_LIMIT) s[i] =  SAFE_LIMIT;
    else if (s[i] < -SAFE_LIMIT) s[i] = -SAFE_LIMIT;
  }
  return n;                            // 항상 꽉 찬 프레임 전송 (부분전송 잡음 방지)
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
  videoFrameCount = 0;
  lastVideoFrame  = -1;
  String vp = videoPathFor(songs[idx]);
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  // ★ FIX10: close 도 반드시 뮤텍스 안에서. 예전엔 여기 close()가 뮤텍스 밖이라
  //   코어0이 SD에서 MP3 읽는 동안 코어1이 SD 파일을 닫아 SPI 버스가 충돌 ->
  //   SD 트랜잭션 깨짐 -> 오디오 소스 읽기 실패 -> 무음. (조이스틱 전환시 간헐)
  if (videoFile) videoFile.close();
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
        // ★ FIX12: 곡 전환을 setPath() '하나로만' 한다. 라이브러리(v1.2.5) 소스 확인:
        //   - player.end() 는 디코더+출력스트림을 tear down 하며 a2dpBuffer 를 간접적
        //     으로 건드림 -> BT콜백 get_data 의 readArray 와 경쟁 -> 데드락(두 코어 정지).
        //   - a2dpBuffer.reset() 도 같은 이유로 get_data 와 경쟁 -> 데드락.
        //   - setPath() 는 active = setStream(selectStream(path)) 로 스스로 active=true
        //     되고 새 파일 디코딩 재개. copy() 는 non-blocking(가득이면 0). 그래서
        //     end()/reset()/프리필/게이트가 전부 불필요 + 유해했음 -> 제거.
        //   버퍼를 안 비우므로 이전 곡 꼬리 ~116ms 잠깐 재생 후 새 곡. 경쟁 자체가 없어
        //   데드락/무음/멈춤이 원천 소멸. (쿠션 유지되니 뽁뽁 위험도 오히려 ↓)
        xSemaphoreTake(sdMutex, portMAX_DELAY);
        bool ok = player.setPath(songs[idx].c_str());
        portENTER_CRITICAL(&clkMux);
        g_bytesPlayed = 0;                     // 영상 시계 새 곡 0 (꼬리만큼 잠깐 앞섬-자가보정)
        portEXIT_CRITICAL(&clkMux);
        xSemaphoreGive(sdMutex);
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
  // ★ FIX9(핵심): SD SPI 클럭 명시. 기본값이 4MHz라 읽기가 느려서 디코더가
  //   버퍼를 못 채우고(=copies 캡), 영상과 SD 경합 -> 언더런('뽁뽁')+fps 하락.
  //   20MHz로 올리면 읽기 ~5배 -> 버퍼 금방 꽉 참 + 영상도 SD 빨리 확보.
  //   (혹시 "SD mount failed" 뜨면 카드가 20MHz를 못 버티는 것 -> 16MHz로 낮출 것)
  if (!SD.begin(SD_CS, SPI, 20000000)) Serial.println("SD mount failed");
  scanSongs();

  sdMutex = xSemaphoreCreateMutex();          // ★ SD 공유 보호

  a2dpBuffer.resize(20 * 1024);        // ★ FIX4: 12KB(68ms)->20KB(~116ms) 쿠션.
                                       //    SD 지연 스파이크 흡수. heap 로그상 최저
                                       //    ~19KB라 20KB(=-4KB) 감당 가능.
  out.begin(95);
  player.setDelayIfOutputFull(0);
  player.setVolume(0.70);
  player.begin(0, false);
  player.setAutoNext(false);

  a2dp.set_data_callback(get_data);
  // ★ FIX13: A2DP 소스 이벤트 태스크를 코어0으로 고정. 기본값이면 이게 코어1(=loop,
  //   영상/I2C)에 얹혀서, 곡 전환 때 영상 SD·I2C 작업에 밀려 소스가 굶고 -> Q30가
  //   미디어 스트림을 서스펜드 -> 전환 후 get_data 정지(뽁뢱)의 원인 후보(#890).
  //   코어0(오디오 디코드)로 옮겨 영상과 분리. 우선순위도 높게.
  a2dp.set_task_core(0);
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

  // ★ 1단계 계측: 2초마다 언더런/버퍼 상태 (영상 유무와 무관하게 항상 출력).
  //   UR>0 & minBuf~0  -> 뽁뽁 = 버퍼 언더런 확정 (2단계로).
  //   UR==0 인데도 뽁뽁 -> 버퍼 문제 아님 (SBC/BT 쪽, 3단계).
  //   vid=N            -> 영상 자체가 안 열림 (mp3<->bin 이름 불일치 등).
  {
    static uint32_t statT = 0;
    uint32_t nowS = millis();
    if (nowS - statT > 2000) {
      statT = nowS;
      uint32_t ur = g_underruns, urb = g_underrunB, mb = g_minBuf, gc = g_getCalls;
      g_underruns = 0; g_underrunB = 0; g_minBuf = 0xFFFFFFFF; g_getCalls = 0;
      AudioInfo ai = decoder.audioInfo();    // ★ 진단: 디코딩된 실제 포맷(레이트/채널)
      Serial.printf("[STAT] UR=%u(%uB) minBuf=%u nowBuf=%u calls=%u conn=%d ast=%d active=%d sr=%u ch=%u pos=%.1fs vid=%s\n",
                    ur, urb, (mb == 0xFFFFFFFF ? 0 : mb),
                    (uint32_t)a2dpBuffer.available(), gc,
                    (int)a2dp.is_connected(), (int)a2dp.get_audio_state(), (int)g_audioActive,
                    ai.sample_rate, ai.channels,
                    g_bytesPlayed / (float)AUDIO_BPS,
                    (videoFile && videoFrameCount) ? "Y" : "N");
    }
  }

  // 곡 끝나면 다음 곡.
  // ★ FIX7: 시작하자마자 오발(1~2초 만에 다음 곡으로 튐) 방지.
  //   원래: songStart를 부팅 때 잡는데 BT 페어링에 몇 초 걸려서, 오디오가
  //   흐르기도 전에 millis()-songStart>3000 이 되고 g_audioActive가 아직
  //   false라 즉시 다음 곡으로 스킵됨(-> 전환마다 버퍼 리셋 -> 뽁뽁).
  //   해결: (a) nowPlaying 이 바뀔 때마다 타이머/래치 리셋(레이스 없음),
  //         (b) 이 곡이 '실제로 재생된 적(sawActive)' 있어야만 종료로 인정.
  static int      lastNP    = -999;
  static uint32_t songStart = 0;
  static bool     sawActive = false;
  if (nowPlaying != lastNP) { lastNP = nowPlaying; songStart = millis(); sawActive = false; }
  if (g_audioActive) sawActive = true;          // 이 곡이 진짜 재생됐다는 증거
  if (songCount > 0 && sawActive && !g_audioActive && millis() - songStart > 3000) {
    shufflePos++;
    if (shufflePos >= songCount) buildShuffle();
    requestSong(shuffleOrder[shufflePos]);      // nowPlaying 갱신 -> 다음 루프서 리셋
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
