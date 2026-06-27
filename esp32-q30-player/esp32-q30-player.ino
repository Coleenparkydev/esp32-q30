#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

#include "AudioTools.h"
#include "AudioTools/Communication/A2DPStream.h"   // brings in BluetoothA2DPSource
#include "AudioTools/Disk/AudioSourceLittleFS.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"

// ---------- SETTINGS ----------
const char* BT_DEVICE_NAME = "Soundcore Life Q30";
const char* AP_SSID        = "ESP32_Music";
const char* AP_PASS        = "";
const char* SONG_PATH      = "/song.mp3";
const int   BOOT_BUTTON    = 0;
// ------------------------------

bool uploadMode = false;
WebServer server(80);
File uploadFile;

// playback chain: LittleFS file -> MP3 decode -> queue -> A2DP callback
AudioSourceLittleFS source("/", "mp3");
MP3DecoderHelix decoder;
BufferRTOS<uint8_t> a2dpBuffer(0);
QueueStream<uint8_t> out(a2dpBuffer);
AudioPlayer player(source, out, decoder);
BluetoothA2DPSource a2dp;

// A2DP pulls PCM from our buffer
int32_t get_data(uint8_t* data, int32_t bytes) {
  return a2dpBuffer.readArray(data, bytes);
}

// ---------------- UPLOAD MODE ----------------
const char* PAGE =
  "<!doctype html><html><head><meta charset='utf-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>ESP32 Music</title>"
  "<style>body{font-family:sans-serif;max-width:420px;margin:40px auto;padding:0 16px}"
  "h2{font-size:1.2rem}button{padding:10px 16px;font-size:1rem}input{margin:12px 0}</style>"
  "</head><body><h2>ESP32 -> Q30 one-song player</h2>"
  "<p>Upload one .mp3 (96 kbps, 44.1 kHz, stereo, under ~2 MB). The board saves it, "
  "reboots, and plays to your Q30.</p>"
  "<form method='POST' action='/upload' enctype='multipart/form-data'>"
  "<input type='file' name='song' accept='.mp3' required><br>"
  "<button type='submit'>Upload &amp; Play</button></form></body></html>";

void handleRoot() { server.send(200, "text/html", PAGE); }

void handleUpload() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    if (LittleFS.exists(SONG_PATH)) LittleFS.remove(SONG_PATH);
    uploadFile = LittleFS.open(SONG_PATH, "w");
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) uploadFile.write(up.buf, up.currentSize);
  } else if (up.status == UPLOAD_FILE_END) {
    if (uploadFile) uploadFile.close();
  }
}

void handleUploadDone() {
  server.send(200, "text/html",
    "<html><body style='font-family:sans-serif;max-width:420px;margin:40px auto'>"
    "<h2>Saved.</h2><p>Rebooting and connecting to your Q30. "
    "Make sure the headphones are ON and in pairing mode.</p></body></html>");
  delay(1500);
  ESP.restart();
}

void startUploadMode() {
  uploadMode = true;
  WiFi.mode(WIFI_AP);
  if (strlen(AP_PASS) == 0) WiFi.softAP(AP_SSID);
  else                      WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/upload", HTTP_POST, handleUploadDone, handleUpload);
  server.begin();
}

// ---------------- PLAY MODE ----------------
void startPlayMode() {
  uploadMode = false;
  WiFi.mode(WIFI_OFF);                 // radio free for Bluetooth only

  a2dpBuffer.resize(20 * 1024);        // 20 KB PCM buffer (no PSRAM on this board)
  out.begin(95);                       // start feeding A2DP when buffer is 95% full
  player.setDelayIfOutputFull(0);
  player.setVolume(0.7);
  player.begin();
  player.setAutoNext(false);           // single file: don't chase a 2nd track

  a2dp.set_data_callback(get_data);
  a2dp.start(BT_DEVICE_NAME);          // connect to the Q30 by name
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);

  if (!LittleFS.begin(true)) Serial.println("LittleFS mount failed");

  bool forceUpload = (digitalRead(BOOT_BUTTON) == LOW);
  bool haveSong    = LittleFS.exists(SONG_PATH);

  if (!haveSong || forceUpload) {
    Serial.println(">> UPLOAD MODE (WiFi ESP32_Music, http://192.168.4.1)");
    startUploadMode();
  } else {
    Serial.println(">> PLAY MODE (connecting to Q30...)");
    startPlayMode();
  }
}

void loop() {
  if (uploadMode) {
    server.handleClient();
  } else {
    player.copy();
    if (!player.isActive()) player.setIndex(0);   // song ended -> restart = loop
  }
}
