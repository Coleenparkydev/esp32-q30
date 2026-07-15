/*
 * CANONICAL MINIMAL A2DP test — pure pschatzmann pattern.
 * No video, no dual-core, no manual switching. loop() = player.copy().
 * setAutoNext(true) lets the library advance songs internally.
 * Goal: does song 2 (auto-advanced) play CLEANLY, or still 뽁뽁?
 *   clean -> base HW/lib is fine, our added complexity is the bug.
 *   pops  -> ESP32 A2DP+SD hardware limit.
 */
#include <SPI.h>
#include <SD.h>
#include "AudioTools.h"
#include "AudioTools/Communication/A2DPStream.h"
#include "AudioTools/Disk/AudioSourceSD.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"

#define SD_CS   13
#define SD_SCK  14
#define SD_MISO  2
#define SD_MOSI 15
#define LORA_CS 18

AudioSourceSD source("/", ".mp3", SD_CS);
MP3DecoderHelix decoder;
BufferRTOS<uint8_t> a2dpBuffer(0);
QueueStream<uint8_t> out(a2dpBuffer);
AudioPlayer player(source, out, decoder);
BluetoothA2DPSource a2dp;

// canonical: return the actual bytes read (no padding)
int32_t get_data(uint8_t* d, int32_t n) {
  return a2dpBuffer.readArray(d, n);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== CANONICAL MINIMAL A2DP test ===");

  pinMode(LORA_CS, OUTPUT); digitalWrite(LORA_CS, HIGH);   // LoRa 라디오 비활성
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (!SD.begin(SD_CS, SPI, 20000000)) Serial.println("SD mount failed");

  a2dpBuffer.resize(20 * 1024);
  out.begin(95);

  player.setDelayIfOutputFull(0);
  player.setVolume(0.5);
  player.setSilenceOnInactive(true);   // A2DP 연결 유지용 무음
  player.setAutoNext(true);            // ★ 라이브러리가 곡을 자동 진행
  player.begin();

  a2dp.set_data_callback(get_data);
  a2dp.start("Soundcore Life Q30");
  Serial.println("started");
}

void loop() {
  player.copy();                       // ★ 이게 전부. 곡 전환도 내부에서 처리.
}
