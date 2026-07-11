/*
 * LoRa32 T3 (v1.6.1 / v2.1) MP3 player -> Soundcore Life Q30 (A2DP)
 * + 동기화된 OLED 뮤직비디오 재생 (SSD1306 128x64, 1비트 .bin)
 *
 * 영상 재생 원리:
 *   - PC 인코더가 만든 songXX.bin (프레임당 1024바이트, SSD1306 버퍼 포맷)을
 *     오디오 songXX.mp3 와 짝으로 SD 루트에 둠.
 *   - get_data(A2DP 콜백)가 소비한 바이트 수(g_bytesPlayed)로 재생 위치를 알아냄.
 *   - 지금 보여줄 프레임 = (재생위치 - 헤드폰지연) / 프레임당바이트.
 *   - 그 프레임을 SD에서 읽어 display.getBuffer()에 직접 넣고 display().
 *   - 오디오가 최우선: OLED blit 직전에 버퍼를 채워 언더런 방지. 밀리면 프레임 드롭.
 *
 * 조이스틱 위/아래 -> 곡 리스트(영상 멈춤), 5초 무동작 시 다시 영상으로.
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
#define AUDIO_BPS       (44100UL * 2 * 2)       // 176400 B/s (44.1kHz 스테레오 16bit)
// 헤드폰/블루투스 지연 보정(ms). 영상이 소리보다 "빠르게" 보이면 값을 올리고,
// 영상이 소리보다 "늦게" 보이면 값을 내려서 귀로 맞추면 됨.
#define SYNC_OFFSET_MS  180
static const uint32_t BYTES_PER_FRAME  = AUDIO_BPS / VIDEO_FPS;              // 14700
static const uint32_t SYNC_OFFSET_BYTES = (AUDIO_BPS * SYNC_OFFSET_MS) / 1000; // ~31752
static const uint16_t FRAME_BYTES = 1024;       // 128x64 / 8
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
volatile uint32_t g_bytesPlayed = 0;   // A2DP가 소비한 PCM 바이트 (재생 위치)
File     videoFile;
uint32_t videoFrameCount = 0;
int32_t  lastVideoFrame = -1;

int32_t get_data(uint8_t* d, int32_t n) {
  int32_t got = a2dpBuffer.readArray(d, n);
  g_bytesPlayed += (uint32_t)got;      // ★ 재생 위치 추적 (마스터 시계)
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

// songXX.mp3 -> songXX.bin (같은 이름, 확장자만 교체)
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

void openVideoFor(int idx) {
  if (videoFile) videoFile.close();
  videoFrameCount = 0;
  lastVideoFrame  = -1;
  String vp = videoPathFor(songs[idx]);
  videoFile = SD.open(vp.c_str(), FILE_READ);
  if (videoFile && !videoFile.isDirectory()) {
    videoFrameCount = videoFile.size() / FRAME_BYTES;
    Serial.printf("video: %s (%u frames)\n", vp.c_str(), videoFrameCount);
  } else {
    if (videoFile) videoFile.close();
    Serial.printf("no video: %s (제목만 표시)\n", vp.c_str());
  }
}

void playSong(int idx) {
  if (idx < 0 || idx >= songCount) return;
  nowPlaying = idx;

  player.end();                                  // 이전 파일 확실히 닫기(누수 방지)
  bool ok = player.setPath(songs[idx].c_str());
  Serial.printf("Playing[%d] %s -> %s\n", idx, songs[idx].c_str(), ok ? "OK" : "FAIL");
  if (!ok) delay(500);                           // 스킵 지옥 방어

  // ★ 영상 싱크 리셋 + 짝 맞는 .bin 열기
  g_bytesPlayed = 0;
  openVideoFor(idx);
}

void playNextShuffle() {
  shufflePos++;
  if (shufflePos >= songCount) buildShuffle();
  playSong(shuffleOrder[shufflePos]);
}

// 오디오 버퍼를 채움(blit/리스트 그리기 직전 언더런 방지). 가득 차면 조기 종료.
inline void topUpAudio(int iters) {
  for (int i = 0; i < iters; i++) if (player.copy() == 0) break;
}

// ---- 영상 한 프레임을 OLED에 (오디오 위치에 맞춰, 밀리면 드롭) ----
void renderVideo() {
  if (!oledOK) return;

  // 이 곡에 영상이 없으면 예전처럼 제목 표시 (바뀔 때만)
  if (!videoFile || videoFrameCount == 0) {
    static int lastT = -2; static bool lastC = false;
    bool c = a2dp.is_connected();
    if (nowPlaying != lastT || c != lastC) {
      topUpAudio(12);
      drawPlayTitle();
      lastT = nowPlaying; lastC = c;
    }
    return;
  }

  uint32_t bp = g_bytesPlayed;
  uint32_t tf = (bp > SYNC_OFFSET_BYTES) ? (bp - SYNC_OFFSET_BYTES) / BYTES_PER_FRAME : 0;
  if (tf >= videoFrameCount) tf = videoFrameCount - 1;   // 끝나면 마지막 프레임 유지
  if ((int32_t)tf == lastVideoFrame) return;             // 같은 프레임 -> 할 일 없음

  topUpAudio(24);                                        // ★ blit 전에 오디오 채우기

  videoFile.seek((uint32_t)tf * FRAME_BYTES);
  int r = videoFile.read(display.getBuffer(), FRAME_BYTES);
  if (r == (int)FRAME_BYTES) display.display();          // 프레임 밀어넣기
  lastVideoFrame = (int32_t)tf;

  // (디버그) 2초마다 실제 fps + 프레임 진행 상황
  static uint32_t dbgT = 0, blits = 0;
  blits++;
  uint32_t nowMs = millis();
  if (nowMs - dbgT > 2000) {
    Serial.printf("fps~%.1f  frame=%d/%u  pos=%.1fs\n",
                  blits / 2.0f, lastVideoFrame, videoFrameCount,
                  bp / (float)AUDIO_BPS);
    blits = 0; dbgT = nowMs;
  }
}

// 영상 없는 곡용 제목 화면 (기존 drawPlay 역할)
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
  Serial.println("\n\n=== MP3 + OLED video player ===");

  pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);
  pinMode(JOY_SW, INPUT_PULLUP);
  analogReadResolution(12);

  // 주의: Adafruit_SSD1306 는 display() 중 I2C 클럭을 자체값(400k)으로 되돌리므로
  // 여기서 1MHz 로 올려도 blit 속도엔 영향 없음. 400k 로 12fps 는 충분함.
  Wire.begin(21, 22); Wire.setClock(400000); Wire.setTimeOut(50);
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() == 0) oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (!SD.begin(SD_CS, SPI)) Serial.println("SD mount failed");
  scanSongs();

  a2dpBuffer.resize(12 * 1024);        // 메모리 안정화 버퍼
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
  // 1. 오디오 버퍼 채우기 (항상, 최우선)
  player.copy();

  // 2. 노래 끝나면 다음 곡
  if (songCount > 0 && !player.isActive()) playNextShuffle();

  uint32_t now = millis();
  static uint32_t lastJoy = 0;
  static bool up = false, down = false;

  if (now - lastJoy >= 50) {
    lastJoy = now;
    int y = analogRead(JOY_Y);
    up = (y > 3300); down = (y < 800);
  }

  bool needsDrawList = false;

  // 조이스틱 위/아래 -> 리스트 모드 (영상 멈춤)
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

  // 버튼 -> 곡 선택 후 영상으로 복귀
  static bool prevSw = HIGH;
  bool sw = digitalRead(JOY_SW);
  if (prevSw == HIGH && sw == LOW) {
    if (uiMode == MODE_LIST) {
      playSong(cursor);            // g_bytesPlayed / lastVideoFrame 리셋됨
      uiMode = MODE_PLAY;
    }
    lastActivity = now;
  }
  prevSw = sw;

  // 5초 무동작 -> 영상으로 복귀
  if (uiMode == MODE_LIST && now - lastActivity > 5000) {
    uiMode = MODE_PLAY;
    lastVideoFrame = -1;           // 복귀 시 현재 프레임 강제 다시 그림
  }

  // ---- 렌더링 ----
  if (uiMode == MODE_LIST) {
    if (needsDrawList) {
      topUpAudio(16);              // 리스트 blit 전에도 오디오 채우기
      drawList();
    }
  } else {
    renderVideo();                 // 영상: 오디오 위치에 맞춰 프레임 표시(드롭 허용)
  }
}
