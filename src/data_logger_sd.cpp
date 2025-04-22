/*
/////////// BASIC microSD DATALOGGER MODULE //////////////////
///////////   MPU6050 & MAX1704X READINGS  ///////////////

- Records multiple 12-second blocks of accelerometer and battery data to microSD
- Number of blocks is configurable via NUM_BLOCKS
- Data is collected at 333Hz to support vibration classification
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
SPIClass sdSPI;
bool max1704xPresent = false;

RTC_DATA_ATTR int sessionFileIndex = 0;
String sessionID;
uint64_t loggingStartTime = 0;
int totalActiveCycles = 0;

String getSessionString(int index) {
  String result = "";
  do {
    result = char('a' + (index % 26)) + result;
    index = index / 26 - 1;
  } while (index >= 0);
  return result;
}

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

bool loadLogStateFromSD(uint64_t &logStart, int &cycles) {
  File file = SD.open(STATE_FILENAME, FILE_READ);
  if (!file) return false;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.startsWith("logStart=")) {
      logStart = strtoull(line.substring(9).c_str(), nullptr, 10);
    } else if (line.startsWith("activeCycles=")) {
      cycles = line.substring(13).toInt();
    }
  }

  file.close();
  return true;
}

void saveLogStateToSD(uint64_t logStart, int cycles) {
  File file = SD.open(STATE_FILENAME, FILE_WRITE);
  if (!file) return;

  file.printf("logStart=%llu\n", logStart);
  file.printf("activeCycles=%d\n", cycles);
  file.close();
}

bool sdHasAccelFiles() {
  File root = SD.open("/");
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    String name = entry.name();
    entry.close();
    if (name.startsWith("accel_sd_") && name.endsWith(".csv")) return true;
  }
  return false;
}

void setup() {
  INIT_DEBUG_SERIAL();
  LOG_DEBUGLN("Booting...");
  delay(100);

  pinMode(VEXT_CTRL_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ALERT_LED_PIN, OUTPUT);
  powerVEXT(true);
  delay(200);

  Wire.begin(SDA_PIN, SCL_PIN);
  LOG_DEBUGLN("I2C reinitialized.");

  if (!testMPU()) {
    LOG_DEBUGLN("ERROR: MPU6050 NOT detected!");
    blinkAlertSensor();
  }

  max1704xPresent = testMAX();
  if (!max1704xPresent) {
    LOG_DEBUGLN("ERROR: MAX1704x NOT detected or disabled!");
    blinkAlertSensor();
  }

  LOG_DEBUGLN("Initializing SD...");
  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sdSPI.setFrequency(400000);

  bool sd_ok = false;
  for (int i = 0; i < 3; i++) {
    if (SD.begin(SD_CS_PIN, sdSPI)) {
      sd_ok = true;
      break;
    }
    delay(500);
  }

  if (!sd_ok) {
    LOG_DEBUGLN("[ERROR] SD failed.");
    blinkAlertError();
    ESP.restart();
  }

  LOG_DEBUGLN("[✓] SD initialized.");
  blinkStatusShort();

  if (!loadLogStateFromSD(loggingStartTime, totalActiveCycles)) {
    loggingStartTime = 0;
    totalActiveCycles = 0;
    saveLogStateToSD(loggingStartTime, totalActiveCycles);
    LOG_DEBUGLN("[INFO] State file not found. Creating new logStart and cycle count.");
  }

  int sessionIndex = 0;
  File indexFile = SD.open("/session_index.txt", FILE_READ);
  if (indexFile) {
    sessionIndex = indexFile.parseInt();
    indexFile.close();
  }
  sessionID = getSessionString(sessionIndex);
  indexFile = SD.open("/session_index.txt", FILE_WRITE);
  if (indexFile) {
    indexFile.printf("%d\n", sessionIndex + 1);
    indexFile.close();
  }

  if (PASSIVE_MODE) {
    uint64_t elapsed = totalActiveCycles * SIMULATED_BATCH_SECONDS;
    uint64_t maxSeconds = TOTAL_LOGGING_HOURS * 3600.0;

    LOG_DEBUG("TotalActiveCycles: %d\n", totalActiveCycles);
    LOG_DEBUG("Elapsed (s):       %llu\n", elapsed);
    LOG_DEBUG("MaxAllowed (s):    %llu\n", maxSeconds);

    if (elapsed >= maxSeconds) {
      LOG_DEBUGLN("✅ Logging window complete. Entering permanent deep sleep.");
      SD.remove(STATE_FILENAME);
      blinkLED(ALERT_LED_PIN, 10, 150, 150);
      esp_deep_sleep_start();
    }

    LOG_DEBUG("Passive Mode - Batch %d\n", totalActiveCycles + 1);
  }

  sessionFileIndex = 0;
  LOG_DEBUG("Session ID: %s\n", sessionID.c_str());

  int blocksToRun = PASSIVE_MODE ? BLOCKS_PER_BATCH : NUM_BLOCKS;
  for (int i = 0; i < blocksToRun; i++) {
    LOG_DEBUG("[✓] Recording block %d of %d\n", i + 1, blocksToRun);
    readSensorsToFile();
    blinkStatusQuick();
    delay(200);
  }

  if (PASSIVE_MODE) {
    totalActiveCycles++;
    saveLogStateToSD(loggingStartTime, totalActiveCycles);
    LOG_DEBUG("Sleeping for %.2f minutes before next batch...\n", BATCH_SLEEP_MINUTES);
    esp_sleep_enable_timer_wakeup((uint64_t)(BATCH_SLEEP_MINUTES * 60.0 * 1e6));
    esp_deep_sleep_start();
  } else {
    blinkStatusSlow();
    delay(500);
    esp_deep_sleep_start();
  }
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

  file.println("timestamp_ms,global_ms,accel_x,accel_y,accel_z,accel_mag,temp_c,voltage,soc,loop_us");
  Serial.printf("Recording to %s...\n", filename.c_str());

  const int fs = 333;
  const int duration = 12;
  const int numSamples = fs * duration;

  unsigned long sessionStart = millis();
  unsigned long sampleIntervalUs = 1000000 / fs;  // 3003 µs
  unsigned long targetMicros = micros();

  for (int i = 0; i < numSamples; i++) {
    unsigned long t0 = micros();

    // Read sensors
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

    float tempC = mpu.getTemperature() / 340.00 + 36.53;
    float accel_mag = sqrt(ax * ax + ay * ay + az * az);
    unsigned long now = millis();
    uint64_t global_ms = (uint64_t)totalActiveCycles * BATCH_DURATION_MS + (now - sessionStart);

    unsigned long t1 = micros();
    unsigned long loop_us = t1 - t0;

    // Write line
    file.printf("%lu,%llu,%d,%d,%d,%.2f,%.2f,%.2f,%.1f,%lu\n",
                now - sessionStart, global_ms,
                ax, ay, az, accel_mag, tempC, voltage, soc, loop_us);

    // Schedule next exact timestamp
    targetMicros += sampleIntervalUs;
    while (micros() < targetMicros);  // Busy wait until next target
  }

  file.close();
  LOG_DEBUGLN("[✓] Recording complete.");
  delay(200);
}