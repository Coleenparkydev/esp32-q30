/*
 * LoRa32 T3 MP3 player -> Soundcore Life Q30 (A2DP) + OLED 뮤직비디오
 *
 * ★ v32 — v30 이후에도 뽂뽂 지속 → 남은 소스측 원인 2개 제거:
 *   (A) 곡 전환마다 helix 디코더 free/malloc(≈29KB) → 힙 단편화 → bluedroid 가
 *       패킷 버퍼 할당 실패 시 조용히 드랍 = 링 무결(P=0)인데도 뽂. (PersistentHelix)
 *   (B) 링 87ms 는 SD 카드 내부 GC 지연 스파이크(수백 ms)를 못 버팀 → 174ms 로 증설,
 *       영상 SYNC_OFFSET 180→267ms 보정. 여전히 나면 P/G/F 로 소스/RF 판별.
 *
 * ★ v30 — A2DPStream 폐기, 직결 구조 복귀 + zero-pad 콜백 (뽂뽂 근본 수정 시도)
 *
 *   라이브러리 소스 정밀 분석(핀 버전: audio-tools v1.2.5 / ESP32-A2DP b559fb15 /
 *   bluedroid IDF5.1) 으로 확정한 3가지:
 *
 *   (1) silence_on_nodata 는 죽은 코드였다. A2DPStream 의 내부 버퍼는
 *       readMaxWait=portMAX_DELAY (BufferRTOS.h:34) 인데 무음 삽입 조건은
 *       readArray()==0 (A2DPStream.h:364). 버퍼가 비면 0 리턴이 아니라
 *       BT 미디어 태스크가 xStreamBufferReceive 안에서 무기한 잠든다.
 *       = "미디어 스트림 stall, get_data 중단"의 진짜 정체 (sink suspend 아님).
 *
 *   (2) 버퍼가 1~511B(빈 건 아님)면 콜백이 partial 을 리턴하는데, bluedroid 는
 *       그걸 residue 로 쌓고(btc_a2dp_source.c:1215) 다음 틱 prep_sbc_2_send 가
 *       PCM 버퍼 앞부분을 memset(0) 으로 덮은 뒤 이어읽는다 -> 이미 받아둔 음악이
 *       무음으로 바뀜 -> 곡 중간 무음 구멍 = 뽂. 기존 maxAvailW 계측은 copy()가
 *       버퍼를 다시 채운 후에만 샘플링해서 이 저수위 순간을 구조적으로 못 봤다.
 *
 *   (3) a2dp setVolume 은 AVRCP 가 아니라 소스에서 PCM 에 지수커브를 직접 곱한다
 *       (0.65 -> x0.233, A2DPVolumeControl.h). 심지어 Q30 볼륨버튼 -> AVRCP 이벤트
 *       -> 라이브러리가 스스로 set_volume() 호출로 켜지기도 한다.
 *
 *   수정: (a) BluetoothA2DPSource 직결 + 자체 링버퍼(BufferRTOS, readWait=0).
 *         (b) 콜백은 절대 블록하지 않고 항상 len 전체 리턴, 부족분은 zero-pad
 *             -> bluedroid 는 underflow/partial 을 영원히 보지 않는다.
 *         (c) A2DPNoVolumeControl 로 라이브러리 PCM 개입 전면 봉인(AVRCP 역주입 포함).
 *             PCM 볼륨은 player.setVolume(0.70) 하나만. ★ 예전보다 많이 커짐 ->
 *             첫 테스트 전에 Q30 볼륨을 먼저 낮출 것!
 *         (d) 진실 지점 계측: 콜백 안에서 최저 수위/pad 발생/호출 간격 +
 *             heap largest free block (단편화 최초 측정).
 */
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "AudioTools.h"
#include "AudioTools/Concurrency/RTOS/BufferRTOS.h"
#include "AudioTools/Disk/AudioSourceSD.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"
#include "BluetoothA2DPSource.h"
#include "esp_heap_caps.h"

// ---------- SETTINGS ----------
// ★ 진단 토글: 1 = SD/디코더 우회하고 440Hz 사인톤을 링버퍼로 직접. 삐- 들리면 링크정상.
#define AUDIO_SELFTEST 0
// ★ 대조실험: 0 = 영상 SD 읽기 끔(제목만 표시).
#define VIDEO_ENABLE 1
const char* BT_DEVICE_NAME = "Soundcore Life Q30";
#define SD_CS   13
#define SD_SCK  14
#define SD_MISO  2
#define SD_MOSI 15
#define LORA_CS 18
#define JOY_Y   34
#define JOY_SW   4
#define MAX_SONGS 150
#define RING_SIZE 30720            // ★v32 174ms @44100/16/2ch — SD 읽기 지연 스파이크(카드 내부 GC, 수백ms급)가 87ms 링을 뚫고 underrun 내는 것 방지

// ---------- VIDEO ----------
#define OLED_ADDR       0x3C
#define VIDEO_FPS       12
#define AUDIO_BPS       (44100UL * 2 * 2)
#define SYNC_OFFSET_MS  267        // ★v32 180+87: 링 2배 → 영상시계(링에 쓴 바이트)의 실제 재생 대비 선행이 87ms 늘어난 만큼 보정
static const uint32_t BYTES_PER_FRAME   = AUDIO_BPS / VIDEO_FPS;
static const uint32_t SYNC_OFFSET_BYTES = (AUDIO_BPS * SYNC_OFFSET_MS) / 1000;
static const uint16_t FRAME_BYTES = 1024;
// ------------------------------

Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOK = false;

// ---- video clock (링버퍼에 쓴 바이트 = 재생 위치; v27 과 동일 기준이라 SYNC_OFFSET 유지) ----
volatile uint32_t g_bytesPlayed = 0;
volatile uint32_t g_wroteBytes  = 0;
static portMUX_TYPE clkMux = portMUX_INITIALIZER_UNLOCKED;

// ---- v30 계측: 전부 '진실 지점'(BT 콜백 내부)에서 측정 ----
volatile int      g_cbMinAvail = RING_SIZE;   // 콜백 진입 시 링버퍼 최저 수위 (512 미만이면 위험지대였단 뜻)
volatile uint32_t g_padEvents  = 0;           // zero-pad 가 발생한 콜백 횟수 (이게 뽁 후보 사건!)
volatile uint32_t g_padBytes   = 0;           // pad 로 채운 총 바이트
volatile uint32_t g_cbMaxGapMs = 0;           // 콜백 호출 간 최장 공백 (>50ms 면 미디어태스크가 굶었다 = stall)
volatile uint32_t g_cbLastMs   = 0;
// ---- v31: 시리얼 없이 OLED 로 읽는 부팅 후 누적 진단 (리셋 안 됨) ----
volatile uint32_t g_padTotal   = 0;           // 부팅 후 pad 사건 총합 (P 숫자)
volatile uint32_t g_cbGapWorst = 0;           // 부팅 후 최악 콜백 공백 ms (G 숫자)
volatile uint32_t g_largestMin = 0xFFFFFFFF;  // 부팅 후 힙 최대연속블록 최저치 (F 숫자)
// 기존 파이프라인 타이밍 계측 (유지)
volatile uint32_t g_maxWriteMs = 0;           // 링버퍼 write 가 블록된 최장 시간
volatile uint32_t g_maxGapMs   = 0;           // copy() 호출 간 최장 공백
volatile uint32_t g_maxMutexMs = 0;           // audioTask 가 sdMutex 기다린 최장 시간
volatile uint32_t g_maxCopyMs  = 0;           // player.copy() 자체가 걸린 최장 시간
volatile uint32_t g_maxVidMs   = 0;           // 코어1이 sdMutex 쥐고 영상 읽은 최장 시간

// ============================================================
//  ★v32 — 곡 전환마다 helix 디코더가 free/malloc(≈29KB) 되는 것 차단.
//  AudioPlayer::setStream() -> end() 가 매 곡 p_decoder->end()/begin() 을 호출하고
//  (AudioPlayer.h:231), libhelix end() 는 MP3FreeDecoder(≈23KB)+버퍼 2개를 해제한다.
//  그 사이 bluedroid 가 20ms 마다 osi_malloc/free 를 하므로 곡을 넘길수록 힙이
//  조각나고, 패킷 버퍼 할당 실패 시 bluedroid 는 '조용히' 프레임을 드랍한다
//  = 링은 멀쩡(P=0)인데 뽂. "7곡쯤 넘기면 심해짐" 관측과 일치. F 숫자가 지표.
// ============================================================
class PersistentHelix : public libhelix::MP3DecoderHelix {
 public:
  // 메모리는 절대 해제하지 않고 스트림 상태만 리셋.
  void end() override {
    frame_buffer.reset();                     // 이전 곡 잔여 바이트 폐기 (free 아님)
    frame_counter = 0;
    active = false;                           // begin() 의 'if(active) end()' 재진입 차단
    memset(&mp3FrameInfo, 0, sizeof(MP3FrameInfo));
    // 원본과 달리 flush() 안 함(이전 곡 꼬리가 링에 섞이는 것 방지),
    // MP3FreeDecoder 안 함. begin() 은 그대로 두면 됨: decoder 가 살아있으면
    // MP3InitDecoder 재호출 없고(allocateDecoder), Vector::resize 는 동일
    // 크기면 no-op → 곡 전환 시 재할당 0회.
  }
};
class PersistentMP3Decoder : public audio_tools::MP3DecoderHelix {
 public:
  PersistentMP3Decoder() {
    delete mp3;                               // 래퍼 기본 드라이버를 지속형으로 교체
    mp3 = new PersistentHelix();
    mp3->setReference(this);
  }
};

// ============================================================
//  오디오 파이프라인: source -> player -> CountingOutput -> audioRing -> BT 콜백
// ============================================================
AudioSourceSD source("/", ".mp3", SD_CS);
PersistentMP3Decoder decoder;
BluetoothA2DPSource a2dp;
BufferRTOS<uint8_t> audioRing(0);             // setup 에서 resize + readWait=0
A2DPNoVolumeControl noVolCtl;                 // 라이브러리 PCM 개입 봉인

// ★ BT 미디어태스크 콜백 — 절대 블록 금지, 항상 len 전체 리턴 (부족분 zero-pad)
int32_t btGetData(uint8_t* data, int32_t len) {
  if (data == nullptr || len <= 0) return 0;
  uint32_t now = millis();
  int avail = audioRing.available();
  int got = audioRing.readArray(data, len);   // readWait=0 -> 즉시 리턴, 블록 없음
  if (got < len) memset(data + got, 0, len - got);
  portENTER_CRITICAL(&clkMux);
  if (avail < g_cbMinAvail) g_cbMinAvail = avail;
  if (got < len) { g_padEvents++; g_padTotal++; g_padBytes += (uint32_t)(len - got); }
  if (g_cbLastMs && now - g_cbLastMs > g_cbMaxGapMs) g_cbMaxGapMs = now - g_cbLastMs;
  if (g_cbMaxGapMs > g_cbGapWorst) g_cbGapWorst = g_cbMaxGapMs;
  g_cbLastMs = now;
  portEXIT_CRITICAL(&clkMux);
  return len;                                 // bluedroid 는 부족을 영원히 모른다
}

// 링버퍼로 들어가는 바이트를 세서 비디오 싱크에 쓰는 얇은 래퍼
class CountingOutput : public AudioOutput {
 public:
  size_t write(const uint8_t* data, size_t len) override {
    uint32_t t0 = millis();
    size_t w = (size_t)audioRing.writeArray(data, len);  // portMAX_DELAY -> 전량 기록 보장
    uint32_t dt = millis() - t0;
    portENTER_CRITICAL(&clkMux);
    g_bytesPlayed += (uint32_t)w;
    g_wroteBytes  += (uint32_t)w;
    if (dt > g_maxWriteMs) g_maxWriteMs = dt;
    portEXIT_CRITICAL(&clkMux);
    return w;
  }
  int availableForWrite() override { return audioRing.availableForWrite(); }
  void setAudioInfo(AudioInfo info) override { AudioOutput::setAudioInfo(info); }
};
CountingOutput counter;
AudioPlayer player(source, counter, decoder);

String songs[MAX_SONGS];
int songCount = 0;
int shuffleOrder[MAX_SONGS];
int shufflePos = 0;

enum { MODE_PLAY, MODE_LIST };
int uiMode = MODE_PLAY;
int cursor = 0;
volatile int nowPlaying = -1;
uint32_t lastMove = 0, lastActivity = 0;

File     videoFile;
uint32_t videoFrameCount = 0;
int32_t  lastVideoFrame = -1;

// ---- 코어 간 공유 ----
SemaphoreHandle_t sdMutex;
volatile bool     g_audioActive     = false;
volatile int      g_reqIndex        = -1;
volatile int      g_videoPendingIdx = -1;
volatile uint32_t g_videoPendingAt  = 0;

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
  g_videoPendingIdx = idx;
  g_videoPendingAt  = millis() + 250;
  // 전환 직후 = 디코더 realloc 직후 -> 단편화 최악 순간을 여기서도 샘플링
  uint32_t lb = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  if (lb < g_largestMin) g_largestMin = lb;
  Serial.printf("Playing[%d] %s\n", idx, songs[idx].c_str());
}

// ============================================================
//  오디오 태스크 (코어0) — 디코딩 + 곡 전환
// ============================================================
void audioTask(void* pv) {
#if AUDIO_SELFTEST
  static int16_t buf[256 * 2];
  float phase = 0.0f;
  const float inc = 2.0f * PI * 440.0f / 44100.0f;
  Serial.println(">>> AUDIO_SELFTEST: 440Hz tone into ring buffer");
  for (;;) {
    for (int i = 0; i < 256; i++) {
      int16_t s = (int16_t)(9000.0f * sinf(phase));
      phase += inc; if (phase > 2.0f * PI) phase -= 2.0f * PI;
      buf[i * 2] = s; buf[i * 2 + 1] = s;
    }
    size_t w = (size_t)audioRing.writeArray((uint8_t*)buf, sizeof(buf));
    portENTER_CRITICAL(&clkMux); g_wroteBytes += (uint32_t)w; portEXIT_CRITICAL(&clkMux);
    g_audioActive = a2dp.is_connected();
    vTaskDelay(1);
  }
#else
  static uint32_t songStart = 0;
  static bool     sawData   = false;
  for (;;) {
    // (1) 전환 요청 처리 (부팅 첫 곡 + 조이스틱 수동선택)
    int req = g_reqIndex;
    if (req >= 0 && req < songCount) {
      g_reqIndex = -1;
      xSemaphoreTake(sdMutex, portMAX_DELAY);
      player.setIndex(req);                     // 인덱스만 선택
      player.setActive(true);                   // ★ setIndex 는 재생을 시작 안 함
      xSemaphoreGive(sdMutex);
      songStart = millis(); sawData = false;
      announceSong(req);
    }

    // (2) 디코딩 -> 링버퍼. 링이 가득이면 writeArray 가 블록 = 자연 페이싱.
    //   미연결시엔 콜백이 안 돌아 링이 안 빠지므로 copy 를 쉰다 (sdMutex 물고 잠들지 않게).
    size_t n = 0;
    bool act = player.isActive();
    if (a2dp.is_connected()) {
      static uint32_t lastCopy = 0;
      uint32_t tc = millis();
      if (lastCopy && tc - lastCopy > g_maxGapMs) g_maxGapMs = tc - lastCopy;
      lastCopy = tc;

      uint32_t tm0 = millis();
      xSemaphoreTake(sdMutex, portMAX_DELAY);
      uint32_t tm1 = millis();               // mutex 대기 끝
      n = player.copy();
      act = player.isActive();
      uint32_t tm2 = millis();               // copy 끝
      xSemaphoreGive(sdMutex);
      portENTER_CRITICAL(&clkMux);
      if (tm1 - tm0 > g_maxMutexMs) g_maxMutexMs = tm1 - tm0;
      if (tm2 - tm1 > g_maxCopyMs)  g_maxCopyMs  = tm2 - tm1;
      portEXIT_CRITICAL(&clkMux);
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    g_audioActive = act;
    if (n > 0) sawData = true;

    // (3) 곡 끝 -> 셔플 다음 곡. autonext=false 이므로 EOF 후 timeoutAutoNext(1.2s)에 active=false.
    if (sawData && !act && g_reqIndex < 0 &&
        a2dp.is_connected() && millis() - songStart > 3000) {
      shufflePos++;
      if (shufflePos >= songCount) buildShuffle();
      int idx = shuffleOrder[shufflePos];
      xSemaphoreTake(sdMutex, portMAX_DELAY);
      player.setIndex(idx);
      player.setActive(true);                   // ★ 다음 곡도 활성화
      xSemaphoreGive(sdMutex);
      songStart = millis(); sawData = false;
      announceSong(idx);
    }

    vTaskDelay(1);
  }
#endif  // AUDIO_SELFTEST
}

void drawPlayTitle() {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(a2dp.is_connected() ? "Q30" : "...");
  // v31: SHUFFLE 대신 누적 진단 — P=pad사건 G=최악콜백공백ms F=힙최대블록최저(KB)
  display.setCursor(44, 0);
  display.printf("P%u G%u F%uk", g_padTotal, g_cbGapWorst,
                 (g_largestMin == 0xFFFFFFFF ? 0 : g_largestMin / 1024));
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  display.setTextWrap(true);
  display.setCursor(0, 18);
  int np = nowPlaying;
  if (np >= 0 && np < songCount) display.print(cleanTitle(songs[np]));
  display.display();
}

void renderVideo() {
  if (!oledOK) return;
#if !VIDEO_ENABLE
  {  // 영상 끔: SD 안 읽고 제목만
    static int lastT = -2; static bool lastC = false;
    bool c = a2dp.is_connected();
    if (nowPlaying != lastT || c != lastC) { drawPlayTitle(); lastT = nowPlaying; lastC = c; }
    return;
  }
#endif
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

  // '이미 그 위치면 seek 생략'은 공짜이고 순차 read-ahead 를 보존하므로 유지.
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  uint32_t tv0 = millis();
  if (videoFile.position() != (uint32_t)tf * FRAME_BYTES)
    videoFile.seek((uint32_t)tf * FRAME_BYTES);
  int r = videoFile.read(display.getBuffer(), FRAME_BYTES);
  uint32_t tvd = millis() - tv0;
  xSemaphoreGive(sdMutex);
  portENTER_CRITICAL(&clkMux);
  if (tvd > g_maxVidMs) g_maxVidMs = tvd;
  portEXIT_CRITICAL(&clkMux);
  if (r == (int)FRAME_BYTES) {
    // v31: 영상 위 진단 오버레이 (시리얼 없이 판독용) — 해결되면 제거
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.printf("P%u G%u F%uk", g_padTotal, g_cbGapWorst,
                   (g_largestMin == 0xFFFFFFFF ? 0 : g_largestMin / 1024));
    display.display();
  }
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
  Serial.println("\n\n=== v32-fable: persistent helix (no per-song free/malloc) + 174ms ring + P/G/F overlay ===");

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

  // ---- 링버퍼 + A2DP 소스 직결 ----
  if (!audioRing.resize(RING_SIZE)) {         // v32: 30KB 연속블록 실패 시 기존 용량으로 후퇴
    Serial.println("ring 30KB alloc FAILED -> fallback 15360 (sync offset will be ~87ms early)");
    audioRing.resize(15360);
  }
  audioRing.setReadMaxWait(0);                 // ★ 콜백은 절대 블록 금지 (v30 핵심 1)
  a2dp.set_volume_control(&noVolCtl);          // ★ 라이브러리 PCM 개입 봉인 (v30 핵심 2)
  a2dp.set_auto_reconnect(true);
  a2dp.set_data_callback(btGetData);
  a2dp.start(BT_DEVICE_NAME);

  source.setTimeoutAutoNext(1200);             // EOF 후 1.2s 뒤 active=false -> 다음 곡
  // delay_if_full=100 기본값 버그 회피 (copy 가 정상 만땅 상태를 '막힘'으로 오인해 100ms 잠듦)
  player.setDelayIfOutputFull(0);
  // 곡 사이/비활성 구간에도 copy() 가 무음을 링에 계속 채움 -> 콜백 pad 가 평시엔 안 뜬다
  player.setSilenceOnInactive(true);
  // PCM 볼륨은 이거 하나. (라이브러리 볼륨 봉인했으므로 예전보다 소리 큼 -> Q30 볼륨 먼저 낮추기!)
  player.setVolume(0.70);
  player.begin(-1, false);                      // 비활성 시작; 첫 곡은 g_reqIndex 로 요청
  player.setAutoNext(false);                    // begin()이 소스값으로 덮으므로 뒤에서 off

  // 디코더 core 0 (원래 설계, 영상 매끄러움)
  xTaskCreatePinnedToCore(audioTask, "audio", 8192, NULL, 2, NULL, 0);

  if (songCount > 0) { buildShuffle(); g_reqIndex = shuffleOrder[0]; }
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
      uint32_t wb, pe, pb, cg, mw, mg, mm, mc, mv;
      int mn;
      portENTER_CRITICAL(&clkMux);
      wb = g_wroteBytes;  g_wroteBytes = 0;
      mn = g_cbMinAvail;  g_cbMinAvail = RING_SIZE;
      pe = g_padEvents;   g_padEvents = 0;
      pb = g_padBytes;    g_padBytes = 0;
      cg = g_cbMaxGapMs;  g_cbMaxGapMs = 0;
      mw = g_maxWriteMs;  g_maxWriteMs = 0;
      mg = g_maxGapMs;    g_maxGapMs = 0;
      mm = g_maxMutexMs;  g_maxMutexMs = 0;
      mc = g_maxCopyMs;   g_maxCopyMs = 0;
      mv = g_maxVidMs;    g_maxVidMs = 0;
      portEXIT_CRITICAL(&clkMux);
      // pad>0 = 콜백이 무음을 채웠다(=예전 같으면 bluedroid underflow 사건).
      // 뽁이 들린 순간과 pad/cbGap 이 같이 뛰면 소스 쪽, 둘 다 0인데 뽁이면 RF/헤드폰 쪽.
      uint32_t lb = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
      if (lb < g_largestMin) g_largestMin = lb;
      Serial.printf("[STAT] np=%d wrote=%uB pos=%.1fs heap=%u largest=%u | ringMin=%d/%d pad=%u/%uB cbGap=%ums | gap=%ums mutex=%ums copy=%ums wr=%ums vid=%ums\n",
                    nowPlaying, wb, g_bytesPlayed / (float)AUDIO_BPS,
                    ESP.getFreeHeap(), lb,
                    mn, RING_SIZE, pe, pb, cg, mg, mm, mc, mw, mv);
    }
  }

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
      g_reqIndex = cursor;
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
