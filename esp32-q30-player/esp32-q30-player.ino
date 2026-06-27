#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

#include "AudioTools.h"
#include "AudioTools/AudioLibs/A2DPStream.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"
#include "AudioTools/Disk/AudioSourceLittleFS.h"

const char* BT_DEVICE_NAME = "Soundcore Life Q30";
const char* AP_SSID        = "ESP32_Music";
const char* AP_PASS        = "";
const char* SONG_PATH      = "/song.mp3";
const int   BOOT_BUTTON    = 0;

bool uploadMode = false;
WebServer server(80);
File uploadFile;

AudioSourceLittleFS source("/", "mp3");
A2DPStream out;
MP3DecoderHelix decoder;
AudioPlayer player(source, out, decoder);

const char* PAGE =
  "<!doctype html><html><head><meta charset='utf-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>ESP32 Music</title>"
  "<style>body{font-family:sans-serif;max-width:420px;margin:40px auto;padding:0 16px}"
  "h2{font-size:1.2rem}button{padding:10px 16px;font-size:1rem}"
  "input{margin:12px 0}</style></head><body>"
  "<h2>ESP32 -> Q30 one-song player</h2>"
  "<p>Upload one .mp3 (96 kbps, under ~2 MB). The board will save it and reboot, "
  "then connect to your Q30 and play.</p>"
  "<form method='POST' action='/upload' enctype='multipart/form-data'>"
  "<input type='file' name='song' accept='.mp3' required><br>"
  "<button type='submit'>Upload &amp; Play</button></form>"
  "</body></html>";

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
    "<h2>Saved.</h2><p>Rebooting and connecting to your Q30 now. "
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

void startPlayMode() {
  uploadMode = false;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  auto cfg = out.defaultConfig(TX_MODE);
  cfg.name = BT_DEVICE_NAME;
  out.begin(cfg);

  player.setAutoNext(true);
  player.begin();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
  }

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
  if (uploadMode) server.handleClient();
  else            player.copy();
}
