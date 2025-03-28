/*
/////////// REAL WORLD VIBRATION DATA LOGGER //////////////////
///////////   MICRO SD  //////////////////////////////////////

- External switch turns on/off device
- Device records 12 sec frames in an infinite loop until micro SD reaches 50% capacity
- Collected data includes accelerometer and battery readings
- Accel data is collected at 1000Hz to fit training classifier purposes 
- Battery data is read at the end of each 12 sec frame and added to the json
- Data is stored in litteFs format, with a timestamp

 
*/

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
#define SD_CS_PIN     33
#define SD_MOSI_PIN   34
#define SD_MISO_PIN   35
#define SD_SCK_PIN    36


MPU6050 mpu;
SFE_MAX1704X lipo;
Preferences prefs;
RTC_DATA_ATTR int bootCount = 0;

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
  LOG_DEBUGLN("Sensors initialized.");
  delay(100);

  Wire.begin(SDA_PIN, SCL_PIN);
  LOG_DEBUGLN("I2C reinitialized.");

  bool mpuOK = testMPU();
  if (!mpuOK) LOG_DEBUGLN("ERROR: MPU6050 NOT detected!");

  bool maxOK = testMAX();
  if (!maxOK) LOG_DEBUGLN("ERROR: MAX1704x NOT detected!");

  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ALERT_LED_PIN, OUTPUT);


  // Initialize SD card
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN);
  if (!SD.begin(SD_CS_PIN)) {
    LOG_DEBUGLN("[ERROR] SD card initialization failed.");
    digitalWrite(ALERT_LED_PIN, HIGH);
    delay(3000);
    digitalWrite(ALERT_LED_PIN, LOW);
    ESP.restart();
  }
  LOG_DEBUGLN("[✔] SD card initialized.");

  prefs.begin("accel", false);
  readSensorsToFile();

  for (int i = 0; i < 3; i++) {   /*visual output: SETUP DONE*/
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(200);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(200);
  }

}

void loop() {}

void powerVEXT(bool state) {
  digitalWrite(VEXT_CTRL_PIN, state ? LOW : HIGH);
}

// void readSensorData() {
//   LOG_DEBUGLN("Reading Sensor Data...");
//   int16_t ax, ay, az, gx, gy, gz;
//   mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//   delay(100);
//   float voltage = lipo.getVoltage();
//   float soc = lipo.getSOC();

//   LOG_DEBUGLN("=== SENSOR DATA ===");
//   Serial.print("{\"accel_x\": "); Serial.print(ax);
//   Serial.print(", \"accel_y\": "); Serial.print(ay);
//   Serial.print(", \"accel_z\": "); Serial.print(az);
//   Serial.print(", \"gyro_x\": "); Serial.print(gx);
//   Serial.print(", \"gyro_y\": "); Serial.print(gy);
//   Serial.print(", \"gyro_z\": "); Serial.print(gz);
//   Serial.print(", \"battery_v\": "); Serial.print(voltage, 2);
//   Serial.print(", \"battery_soc\": "); Serial.print(soc, 1);
//   Serial.println("}");

// }

bool testMPU() {
  mpu.initialize();
  return mpu.testConnection();
}

bool testMAX() {
  return lipo.begin();
}

String getNextFilename() {
  int count = prefs.getInt("fileCount", 0);
  count++;
  prefs.putInt("fileCount", count);
  return "/accel" + String(count) + ".csv";
}

/*
TODO
- find out need for switch off gyro
- */
void readSensorsToFile() {
  String filename = getNextFilename();
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("[ERROR] Could not open file on SD card.");
    return;
  }

  file.println("timestamp_ms,accel_x,accel_y,accel_z");
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
      file.printf("%lu,%d,%d,%d\n", voltage, soc, now - startTime, ax, ay, az);
      nextSample += interval;
    }
  }

  file.close();
  LOG_DEBUGLN("[✔] Recording complete.");
  delay(200);
}
