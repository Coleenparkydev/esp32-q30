/*
 * LoRa32 T3 MP3 player -> Soundcore Life Q30 (A2DP) + OLED 뮤직비디오
 *
 * ★ v4 — A2DPStream 정석 아키텍처 (뽁뢱 근본 해결)
 *   지난 문제: 직접 BluetoothA2DPSource + get_data + BufferRTOS 를 손으로 굴렸다.
 *     곡 전환 때 이 수제 파이프라인이 A2DP 소스를 흔들어 스트림이 정지(suspend)->뽁뢱.
 *     (첫 곡만 깨끗했던 이유 = 연결 직후 fresh 상태)
 *   해결: 라이브러리 메인테이너(pschatzmann, discussion #998)가 지정한 정석 =
 *     AudioPlayer 의 출력을 A2DPStream 으로 두고 silence_on_nodata=true.
 *     -> 전환 갭에도 A2DP 콜백이 무음을 흘려보내서 스트림이 절대 안 끊긴다 = 팝 방지.
 *     + player.setAutoFade(true) 로 곡 경계 페이드(클릭 제거).
 *     + source.setTimeoutAutoNext(1200): 예전 30000이 원인이라 다음 곡이 안 넘어갔음
 *       (EOF 후 active가 30초 뒤에나 꺼짐). 1.2s로 줄여 자동 넘김 정상화.
 *   비디오 싱크: CountingOutput 이 A2DPStream 으로 보내는 바이트를 세서 재생위치로 사용.
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
// ★ 진단 토글: 1 = SD/디코더 우회하고 440Hz 사인톤을 A2DP로 직접 송출.
//   삐- 들리면 BT링크 정상(문제는 디코드경로), 무음이면 Q30링크 문제.
//   0 = 실제 재생(정상 모드). setActive 버그 수정 후엔 0으로 두고 이걸로 플래시.
//   만약 0에서도 무음이면 → 1로 바꿔 재플래시해서 '삐-' 들리는지로 링크/디코드 판별.
#define AUDIO_SELFTEST 0
const char* BT_DEVICE_NAME = "Soundcore Life Q30";
#define SD_CS   13
#define SD_SCK  14
#define SD_MISO  2
#define SD_MOSI 15
#define LORA_CS 18
#define JOY_Y   34
#define JOY_SW   4
#define MAX_SONGS 150

// ---------- VIDEO ----------
#define OLED_ADDR       0x3C
#define VIDEO_FPS       12
#define AUDIO_BPS       (44100UL * 2 * 2)
#define SYNC_OFFSET_MS  180
static const uint32_t BYTES_PER_FRAME   = AUDIO_BPS / VIDEO_FPS;
static const uint32_t SYNC_OFFSET_BYTES = (AUDIO_BPS * SYNC_OFFSET_MS) / 1000;
static const uint16_t FRAME_BYTES = 1024;
// ------------------------------

Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOK = false;

// ---- video clock (오디오태스크가 A2DP로 보낸 바이트 = 재생 위치) ----
volatile uint32_t g_bytesPlayed = 0;
volatile uint32_t g_wroteBytes  = 0;          // STAT용: 실제로 흘러간 오디오 바이트
static portMUX_TYPE clkMux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================
//  오디오 파이프라인 (라이브러리 정석: source -> player -> A2DPStream)
// ============================================================
AudioSourceSD source("/", ".mp3", SD_CS);
MP3DecoderHelix decoder;
A2DPStream a2dp_out;                           // 플레이어 최종 출력 = A2DP 소스

// A2DPStream 으로 나가는 바이트를 세서 비디오 싱크에 쓰는 얇은 래퍼
class CountingOutput : public AudioOutput {
 public:
  size_t write(const uint8_t* data, size_t len) override {
    size_t w = a2dp_out.write(data, len);      // 버퍼 가득차면 블로킹 = 실시간 속도조절
    portENTER_CRITICAL(&clkMux);
    g_bytesPlayed += (uint32_t)w;
    g_wroteBytes  += (uint32_t)w;
    portEXIT_CRITICAL(&clkMux);
    return w;
  }
  int availableForWrite() override { return a2dp_out.availableForWrite(); }
  void setAudioInfo(AudioInfo info) override {
    AudioOutput::setAudioInfo(info);
    a2dp_out.setAudioInfo(info);
  }
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
volatile int nowPlaying = -1;                 // audioTask -> loop (현재 곡 인덱스)
uint32_t lastMove = 0, lastActivity = 0;

File     videoFile;
uint32_t videoFrameCount = 0;
int32_t  lastVideoFrame = -1;

// ---- 코어 간 공유 ----
SemaphoreHandle_t sdMutex;
volatile bool     g_audioActive     = false;  // audioTask -> loop
volatile int      g_reqIndex        = -1;     // loop -> audioTask: 이 곡으로 전환 요청
volatile int      g_videoPendingIdx = -1;     // audioTask -> loop: 이 곡 영상 열어라
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
  g_videoPendingIdx = idx;                     // 코어1이 살짝 뒤에 영상 오픈
  g_videoPendingAt  = millis() + 250;
  Serial.printf("Playing[%d] %s\n", idx, songs[idx].c_str());
}

// ============================================================
//  오디오 태스크 (코어0) — 디코딩 + 곡 전환을 '한 태스크에서' 처리
//  전환은 player.setIndex() 뿐. A2DPStream 은 그대로 두면 무음이 계속 흘러 안 끊긴다.
// ============================================================
void audioTask(void* pv) {
#if AUDIO_SELFTEST
  // 440Hz 사인톤을 A2DP로 직접 (디코더/SD/플레이어 전부 우회) — 링크 검증용
  static int16_t buf[256 * 2];
  float phase = 0.0f;
  const float inc = 2.0f * PI * 440.0f / 44100.0f;
  Serial.println(">>> AUDIO_SELFTEST: 440Hz tone direct to A2DP");
  for (;;) {
    for (int i = 0; i < 256; i++) {
      int16_t s = (int16_t)(9000.0f * sinf(phase));
      phase += inc; if (phase > 2.0f * PI) phase -= 2.0f * PI;
      buf[i * 2] = s; buf[i * 2 + 1] = s;
    }
    size_t w = a2dp_out.write((uint8_t*)buf, sizeof(buf));
    portENTER_CRITICAL(&clkMux); g_wroteBytes += (uint32_t)w; portEXIT_CRITICAL(&clkMux);
    g_audioActive = a2dp_out.isConnected();
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
      player.setIndex(req);                     // 인덱스만 선택 (활성화 안 됨!)
      player.setActive(true);                   // ★ FIX: setIndex는 재생을 시작 안 함. 이게 없어서 무음이었다.
      xSemaphoreGive(sdMutex);
      songStart = millis(); sawData = false;
      announceSong(req);
    }

    // (2) 디코딩 -> A2DP. copy()는 A2DP 버퍼가 가득이면 write에서 블로킹(실시간).
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    size_t n = player.copy();
    bool act = player.isActive();
    xSemaphoreGive(sdMutex);
    g_audioActive = act;
    if (n > 0) sawData = true;

    // (3) 곡 끝 -> 셔플 다음 곡.
    //     autonext=false 이므로 EOF 후 timeoutAutoNext(1.2s) 지나면 active=false 로 떨어짐.
    if (sawData && !act && g_reqIndex < 0 &&
        a2dp_out.isConnected() && millis() - songStart > 3000) {
      shufflePos++;
      if (shufflePos >= songCount) buildShuffle();
      int idx = shuffleOrder[shufflePos];
      xSemaphoreTake(sdMutex, portMAX_DELAY);
      player.setIndex(idx);
      player.setActive(true);                   // ★ FIX: 다음 곡도 명시적으로 활성화
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
  display.print(a2dp_out.isConnected() ? "Q30" : "...");
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
    bool c = a2dp_out.isConnected();
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
  Serial.println("\n\n=== MP3 + OLED player (v4: A2DPStream) ===");

  pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);
  pinMode(JOY_SW, INPUT_PULLUP);
  analogReadResolution(12);

  Wire.begin(21, 22); Wire.setClock(400000); Wire.setTimeOut(50);
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() == 0) oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  // SD 는 반드시 20MHz 로 먼저 마운트 (이미 마운트되면 source.begin()이 재마운트 안 함 -> 속도 유지)
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (!SD.begin(SD_CS, SPI, 20000000)) Serial.println("SD mount failed");
  scanSongs();

  sdMutex = xSemaphoreCreateMutex();

  // ---- A2DP 출력 (플레이어 sink) ----
  // silence_on_nodata=true : 디코드 버퍼가 잠깐 비어도(=곡 전환 갭) A2DP 콜백이 무음을
  //   내보내 스트림을 살아있게 유지 -> 2번째 곡부터 나던 뽁뢱의 근본 해결.
  A2DPConfig cfg = a2dp_out.defaultConfig(TX_MODE);
  cfg.name = BT_DEVICE_NAME;
  cfg.silence_on_nodata = true;
  cfg.auto_reconnect = true;
  cfg.wait_for_connection = false;             // 연결 대기로 setup/loop 막지 않음 (UI 바로 살아남)
  a2dp_out.setVolume(0.65);
  a2dp_out.begin(cfg);

  source.setTimeoutAutoNext(1200);             // ★ EOF 후 1.2s 뒤 active=false -> 다음 곡 (예전 30000이 버그)
  player.setSilenceOnInactive(true);           // 갭 동안 무음 (2차 안전망)
  player.setVolume(1.0);                        // 실제 볼륨은 A2DP 단(0.65)에서
  player.begin(-1, false);                      // 비활성 시작; 첫 곡은 g_reqIndex 로 요청
  player.setAutoFade(true);                     // 전환 페이드 = 클릭/팝 제거
  player.setAutoNext(false);                    // ★ begin()이 소스값으로 덮으므로 뒤에서 off (셔플 직접관리)

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
      uint32_t wb; portENTER_CRITICAL(&clkMux); wb = g_wroteBytes; g_wroteBytes = 0; portEXIT_CRITICAL(&clkMux);
      Serial.printf("[STAT] conn=%d act=%d np=%d wrote=%uB bufFree=%d pos=%.1fs\n",
                    (int)a2dp_out.isConnected(), (int)g_audioActive, nowPlaying,
                    wb, a2dp_out.availableForWrite(), g_bytesPlayed / (float)AUDIO_BPS);
    }
  }

  // audioTask가 곡을 바꿨으면 영상 오픈
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
