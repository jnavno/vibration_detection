#include <Wire.h>
#include <MPU6050.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <esp_sleep.h>
#include <Preferences.h>
#include <driver/touch_pad.h>
#include <DebugConfiguration.h>
#include "variant.h"
#include <SD.h>
#include <SPI.h>

// SD card pins on Heltec V3 (custom mapping)
#define SD_CS_PIN     19   // Still a solid choice
#define SD_MOSI_PIN   38   // Safe
#define SD_MISO_PIN   39   // Safe
#define SD_SCK_PIN    37   // Safe


MPU6050 mpu;
SFE_MAX1704X lipo;
Preferences prefs;
RTC_DATA_ATTR int bootCount = 0;
SPIClass sdSPI;

// Utility macro for debugging pin levels
#define DUMP_SD_PINS() do { \
  Serial.printf("CS: %d, MOSI: %d, MISO: %d, SCK: %d\n", \
    digitalRead(SD_CS_PIN), digitalRead(SD_MOSI_PIN), \
    digitalRead(SD_MISO_PIN), digitalRead(SD_SCK_PIN)); \
} while(0)

void powerVEXT(bool state);
bool testMPU();
bool testMAX();
void readSensorsToFile();
String getNextFilename();

void setup() {
  INIT_DEBUG_SERIAL();
  LOG_DEBUGLN("Booting...");
  delay(100);

  pinMode(VEXT_CTRL_PIN, OUTPUT);
  powerVEXT(true);
  delay(200); // Added: Allow peripherals to stabilize

  LOG_DEBUGLN("Sensors initialized.");

  Wire.begin(SDA_PIN, SCL_PIN);
  LOG_DEBUGLN("I2C reinitialized.");

  bool mpuOK = testMPU();
  if (!mpuOK) LOG_DEBUGLN("ERROR: MPU6050 NOT detected!");

  bool maxOK = testMAX();
  if (!maxOK) LOG_DEBUGLN("ERROR: MAX1704x NOT detected!");

  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ALERT_LED_PIN, OUTPUT);

  LOG_DEBUGLN("Initializing SD...");
  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sdSPI.setFrequency(400000);  // Reduced for stability

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
    DUMP_SD_PINS();  // Show SPI pin states
    for (int i = 0; i < 3; i++) {
      digitalWrite(ALERT_LED_PIN, HIGH); delay(300);
      digitalWrite(ALERT_LED_PIN, LOW); delay(300);
    }
    ESP.restart();
  }

  LOG_DEBUGLN("[✔] SD card initialized.");
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

  prefs.begin("accel", false);
  readSensorsToFile();

  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH); delay(200);
    digitalWrite(STATUS_LED_PIN, LOW); delay(200);
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
  bool ok = lipo.begin();
  if (!ok) {
    LOG_DEBUGLN("[MAX1704x] begin() failed.");
    return false;
  }

  LOG_DEBUG("[MAX1704x] Voltage: ");
  LOG_DEBUGLN(lipo.getVoltage(), 2);
  LOG_DEBUG("[MAX1704x] SOC: ");
  LOG_DEBUGLN(lipo.getSOC(), 2);
  return true;
}

String getNextFilename() {
  int count = prefs.getInt("fileCount", 0);
  count++;
  prefs.putInt("fileCount", count);
  return "/accel" + String(count) + ".csv";
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
      float voltage = lipo.getVoltage();
      float soc = lipo.getSOC();
      unsigned long now = millis();
      file.printf("%lu,%d,%d,%d,%.2f,%.1f\n", now - startTime, ax, ay, az, voltage, soc);
      nextSample += interval;
    }
  }

  file.close();
  LOG_DEBUGLN("[✔] Recording complete.");
  delay(200);
}
