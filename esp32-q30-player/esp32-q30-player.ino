/*
 * RFID UID reader for LoRa32 T3 (v1.6.1 / v2.1) + RC522 + OLED
 * Tap each of your 6 cards; its UID appears on the OLED and on Serial @115200.
 * Write down the 6 UIDs - you'll use them as SD-card folder names for the jukebox.
 *
 * Wiring (RC522 -> LoRa32):
 *   SDA/SS -> IO4,  SCK -> IO14,  MOSI -> IO15,  MISO -> IO2
 *   RST    -> 3.3V (tied high, soft reset),  3.3V -> 3.3V,  GND -> GND,  IRQ -> (nc)
 */

#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define RC522_CS 4
#define SD_SCK  14
#define SD_MISO  2
#define SD_MOSI 15

MFRC522 mfrc(RC522_CS, MFRC522::UNUSED_PIN);      // CS=4, no hardware RST (soft reset)
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOK = false;

void initOLED() {
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Wire.setTimeOut(50);
  Wire.beginTransmission(0x3C);
  if (Wire.endTransmission() == 0) {
    oledOK = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  }
  Serial.printf("OLED: %s\n", oledOK ? "found 0x3C" : "not found");
}

void show(const String& line1, const String& line2) {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(line1);
  display.setTextSize(2);
  display.setCursor(0, 24);
  display.println(line2);
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== RFID UID READER ===");

  initOLED();

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);            // RC522 shares the SD SPI pins
  mfrc.PCD_Init();
  delay(50);

  // sanity check: read the RC522 version register (0x91/0x92 = OK, 0x00/0xFF = wiring problem)
  byte v = mfrc.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.printf("RC522 version: 0x%02X %s\n", v,
                (v == 0x91 || v == 0x92) ? "(OK)" : "(CHECK WIRING!)");

  show("Tap a card...", "");
  Serial.println("Tap each card to read its UID.");
}

void loop() {
  if (!mfrc.PICC_IsNewCardPresent() || !mfrc.PICC_ReadCardSerial()) {
    delay(50);
    return;
  }

  String uid = "";
  for (byte i = 0; i < mfrc.uid.size; i++) {
    if (mfrc.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(mfrc.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();

  Serial.println(">> CARD UID: " + uid);
  show("Card UID:", uid);

  mfrc.PICC_HaltA();
  delay(1200);
}
