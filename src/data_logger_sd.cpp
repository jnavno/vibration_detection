/*
/////////// BASIC microSD DATALOGGER MODULE //////////////////
///////////   MPU6050 & MAX1704X READINGS  ///////////////

- Records multiple 12-second blocks of accelerometer and battery data to microSD
- Number of blocks is configurable via NUM_BLOCKS
- Data is collected at 1000Hz to support vibration classification
- Accelerometer readings include X, Y, Z axes only (no gyroscope)
- Battery readings include voltage and SOC via MAX17048
- Collected data is saved to CSV files: /accel_sd_<session><index>.csv
- SD card is initialized over custom software SPI pins
- VEXT power rail is toggled ON to power sensors
- Debugging is controlled via DEBUG_LEVEL in DebugConfiguration.h
*/

/*
/////////// LED FEEDBACK LEGEND //////////////////

Purpose                     LED     Pattern               Meaning
---------------------------------------------------------------------
✅ SD card initialized      STATUS  3 short blinks        System ready to log
🔴 SD init failed           ALERT   3 slow blinks         Error condition, system restarting
🟨 Each 12s block finished  STATUS  1 very quick blink    Block saved to microSD
❌ Sensor not detected      ALERT   5 short blinks        Warning: sensor problem (MPU or MAX)
✅ Final completion         STATUS  3 slow blinks         All blocks complete
---------------------------------------------------------------------
See blinkLED(), blinkStatus*, and blinkAlert* for implementation.
*/

#include <Wire.h>
#include <MPU6050.h>
#ifdef USE_MAX1704X
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#endif
#include <esp_sleep.h>
#include <Preferences.h>
#include <driver/touch_pad.h>
#include <DebugConfiguration.h>
#include "variant.h"
#include <SD.h>
#include <SPI.h>

MPU6050 mpu;
#ifdef USE_MAX1704X
SFE_MAX1704X lipo;
#endif
Preferences prefs;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int sessionFileIndex = 0;
SPIClass sdSPI;
bool max1704xPresent = false;
String sessionID;

// Convert integer to session string (a, b, ..., z, aa, ab, ...)
String getSessionString(int index) {
  String result = "";
  do {
    result = char('a' + (index % 26)) + result;
    index = index / 26 - 1;
  } while (index >= 0);
  return result;
}

// Modular LED Blink Utilities
void blinkLED(int pin, int blinks, int on_ms, int off_ms) {
  for (int i = 0; i < blinks; i++) {
    digitalWrite(pin, HIGH);
    delay(on_ms);
    digitalWrite(pin, LOW);
    delay(off_ms);
  }
}

void blinkStatusShort()    { blinkLED(STATUS_LED_PIN, 3, 100, 100); }
void blinkStatusQuick()    { blinkLED(STATUS_LED_PIN, 1, 40, 40); }
void blinkStatusSlow()     { blinkLED(STATUS_LED_PIN, 3, 300, 200); }
void blinkAlertError()     { blinkLED(ALERT_LED_PIN, 3, 300, 300); }
void blinkAlertSensor()    { blinkLED(ALERT_LED_PIN, 5, 100, 100); }

void powerVEXT(bool state);
bool testMPU();
bool testMAX();
void readSensorsToFile();
String getNextFilename();
bool sdHasAccelFiles();


void setup() {
  INIT_DEBUG_SERIAL();
  LOG_DEBUGLN("Booting...");
  delay(100);

  pinMode(VEXT_CTRL_PIN, OUTPUT);
  powerVEXT(true);
  delay(200);

  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ALERT_LED_PIN, OUTPUT);

  LOG_DEBUGLN("Sensors initialized.");

  Wire.begin(SDA_PIN, SCL_PIN);
  LOG_DEBUGLN("I2C reinitialized.");

  bool mpuOK = testMPU();
  if (!mpuOK) LOG_DEBUGLN("ERROR: MPU6050 NOT detected!");
  blinkAlertSensor();

  bool maxOK = testMAX();
  if (!max1704xPresent) LOG_DEBUGLN("ERROR: MAX1704x NOT detected or disabled!");
  blinkAlertSensor();

  LOG_DEBUGLN("Initializing SD...");
  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sdSPI.setFrequency(400000);

  bool sd_ok = false;
  for (int attempt = 1; attempt <= 3; attempt++) {
    LOG_DEBUG("Trying SD.begin() attempt "); LOG_DEBUGLN(attempt);
    if (SD.begin(SD_CS_PIN, sdSPI)) {
      sd_ok = true;
      break;
    }
    delay(500);
  }

  if (!sd_ok) {
    LOG_DEBUGLN("[ERROR] SD.begin() failed after retries.");
    DUMP_SD_PINS();
    blinkAlertError();
    for (int i = 0; i < 3; i++) {
      digitalWrite(ALERT_LED_PIN, HIGH); delay(300);
      digitalWrite(ALERT_LED_PIN, LOW); delay(300);
    }
    ESP.restart();
  }

  LOG_DEBUGLN("[\u2713] SD card initialized.");
  LOG_DEBUG("Using SPI pins — CS="); LOG_DEBUGLN(SD_CS_PIN);
  LOG_DEBUG(", MOSI="); LOG_DEBUGLN(SD_MOSI_PIN);
  LOG_DEBUG(", MISO="); LOG_DEBUGLN(SD_MISO_PIN);
  LOG_DEBUG(", SCK="); LOG_DEBUGLN(SD_SCK_PIN);

  File file = SD.open("/test.txt", FILE_WRITE);
  if (file) {
    file.println("Hello SD card!");
    file.close();
    Serial.println("Write successful.");
  } else {
    Serial.println("Failed to open file.");
  }

  blinkStatusShort();

  prefs.begin("accel", false);
  if (!sdHasAccelFiles()) {
    LOG_DEBUGLN("[Info] No accel_sd_*.csv files found — resetting sessionIndex.");
    prefs.putInt("sessionIndex", 0);
  }
  int sessionIndex = prefs.getInt("sessionIndex", 0);
  sessionID = getSessionString(sessionIndex);
  prefs.putInt("sessionIndex", sessionIndex + 1);
  prefs.end();

  sessionFileIndex = 0;
  LOG_DEBUG("Session ID: %s\n", sessionID.c_str());

  for (int i = 0; i < NUM_BLOCKS; i++) {
    LOG_DEBUG("[\u2713] Recording block %d of %d\n", i + 1, NUM_BLOCKS);
    readSensorsToFile();
    blinkStatusQuick();
    delay(200);
  }

  blinkStatusSlow();

  LOG_DEBUGLN("All recordings completed. Entering deep sleep now...");
  delay(500);
  esp_deep_sleep_start();

}

bool sdHasAccelFiles() {
  File root = SD.open("/");
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;

    String name = entry.name();
    entry.close();
    if (name.startsWith("accel_sd_") && name.endsWith(".csv")) {
      return true;
    }
  }
  return false;
}


void loop() {}

void powerVEXT(bool state) {
  digitalWrite(VEXT_CTRL_PIN, state ? LOW : HIGH);
}

bool testMPU() {
  mpu.initialize();
  return mpu.testConnection();
}

bool testMAX() {
#ifdef USE_MAX1704X
  LOG_DEBUGLN("Initializing MAX17048...");
  if (!lipo.begin()) {
    LOG_DEBUGLN("[MAX1704x] begin() failed — sensor not detected.");
    return false;
  }
  delay(200);
  Wire.beginTransmission(0x36);
  Wire.write(0x08);
  Wire.endTransmission(false);
  Wire.requestFrom(0x36, 2);
  uint16_t version = (Wire.read() << 8) | Wire.read();
  LOG_DEBUG("[MAX1704x] Version: 0x%04X\n", version);

  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom(0x36, 2);
  uint16_t config = (Wire.read() << 8) | Wire.read();
  bool sleeping = config & (1 << 7);
  LOG_DEBUG("[MAX1704x] CONFIG: 0x%04X — Sleeping: %s\n", config, sleeping ? "Yes" : "No");

  Wire.beginTransmission(0x36);
  Wire.write(0xFE);
  Wire.write(0x00);
  Wire.write(0x54);
  Wire.endTransmission();
  LOG_DEBUGLN("[MAX1704x] Reset command sent.");

  delay(1000);
  if (!lipo.begin()) {
    LOG_DEBUGLN("[MAX1704x] Re-init after reset failed.");
    return false;
  }

  LOG_DEBUGLN("[MAX1704x] Monitoring initial SOC...");
  for (int i = 0; i < 5; i++) {
    float voltage = lipo.getVoltage();
    float soc = lipo.getSOC();
    LOG_DEBUG("[MAX1704x] Voltage: %.2f V, SOC: %.2f %%\n", voltage, soc);
    delay(1000);
  }
  return true;
#else
  return false;
#endif
}

String getNextFilename() {
  sessionFileIndex++;
  return "/accel_sd_" + sessionID + String(sessionFileIndex) + ".csv";
}

void readSensorsToFile() {
  String filename = getNextFilename();
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.printf("[ERROR] Could not open file: %s\n", filename.c_str());
    return;
  }
  file.println("timestamp_ms,accel_x,accel_y,accel_z,voltage,soc");
  Serial.printf("Recording to %s...\n", filename.c_str());

  unsigned long startTime = millis();
  unsigned long interval = 1000 / SAMPLE_RATE_ACCEL_HZ;
  unsigned long nextSample = startTime;

  while (millis() - startTime < SAMPLING_DURATION_ACCEL * 1000) {
    if (millis() >= nextSample) {
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      float voltage = -1.0f;
      float soc = -1.0f;
#ifdef USE_MAX1704X
      if (max1704xPresent) {
        voltage = lipo.getVoltage();
        soc = lipo.getSOC();
      }
#endif

      unsigned long now = millis();
      file.printf("%lu,%d,%d,%d,%.2f,%.1f\n", now - startTime, ax, ay, az, voltage, soc);
      nextSample += interval;
    }
  }

  file.close();
  LOG_DEBUGLN("[\u2713] Recording complete.");
  delay(200);
}
