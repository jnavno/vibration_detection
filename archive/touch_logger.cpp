#include <Wire.h>
#include <MPU6050.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <esp_sleep.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <driver/touch_pad.h>

#define SDA_PIN 41
#define SCL_PIN 42
#define VEXT_CTRL_PIN 36
#define STATUS_LED_PIN 4
#define ALERT_LED_PIN 5
#define TOUCH_GPIO 2
#define TOUCH_PAD_NUM TOUCH_PAD_NUM2  // GPIO2
#define SAMPLE_RATE_HZ 1000
#define DURATION_SEC 12

#if CONFIG_IDF_TARGET_ESP32
#define TOUCH_THRESHOLD 40   // Greater = more sensitive
#else
#define TOUCH_THRESHOLD 10000  // Lower = more sensitive (adjusted for ESP32-S3)
#endif

MPU6050 mpu;
SFE_MAX1704X lipo;
Preferences prefs;
RTC_DATA_ATTR int bootCount = 0;
touch_pad_t touchPin;

void powerVEXT(bool state);
bool testMPU();
bool testMAX();
void enterDeepSleep();
void recordAccelToFile();
String getNextFilename();
void printWakeupReason();
void printWakeupTouchpad();

void setup() {
  Serial.begin(115200);
  delay(100);

  bootCount++;
  Serial.printf("Boot count: %d\n", bootCount);

  printWakeupReason();
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TOUCHPAD) {
    esp_err_t status;
    touchPin = esp_sleep_get_touchpad_wakeup_status();
    if (touchPin != TOUCH_PAD_MAX && touchPin != TOUCH_PAD_MAX - 1) {
      printWakeupTouchpad();
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(500);
      digitalWrite(STATUS_LED_PIN, LOW);
    } else {
      Serial.println("Wakeup from touchpad, but status invalid or unavailable.");
    }
  }

  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(VEXT_CTRL_PIN, OUTPUT);
  powerVEXT(true);
  delay(100);

  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ALERT_LED_PIN, OUTPUT);

  if (!testMPU()) Serial.println("MPU6050 not found");
  if (!testMAX()) Serial.println("MAX1704x not found");

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
    Serial.printf("Filesystem total: %u bytes\n", LittleFS.totalBytes());
    Serial.printf("Filesystem used: %u bytes\n", LittleFS.usedBytes());
    Serial.println("Formatting LittleFS...");
    LittleFS.format();
    delay(1000);
    digitalWrite(ALERT_LED_PIN, HIGH);
    delay(1000);
    digitalWrite(ALERT_LED_PIN, LOW);
    ESP.restart();
  }

  prefs.begin("accel", false);
  recordAccelToFile();

  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(200);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(200);
  }

  powerVEXT(false);
  enterDeepSleep();
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
  return lipo.begin();
}

String getNextFilename() {
  int count = prefs.getInt("fileCount", 0);
  count++;
  prefs.putInt("fileCount", count);
  return "/accel" + String(count) + ".csv";
}

void recordAccelToFile() {
  const size_t estimatedFileSize = SAMPLE_RATE_HZ * DURATION_SEC * 20;
  size_t freeSpace = LittleFS.totalBytes() - LittleFS.usedBytes();

  Serial.printf("LittleFS free space: %u bytes\n", freeSpace);

  if (freeSpace < estimatedFileSize + 1024) {
    Serial.println("[WARNING] Insufficient space to record new data.");
    Serial.println("-> Please connect to your computer, extract, and delete old files via serial or mount as mass storage.");
    digitalWrite(ALERT_LED_PIN, HIGH);
    delay(3000);
    digitalWrite(ALERT_LED_PIN, LOW);
    return;
  }

  String filename = getNextFilename();
  File file = LittleFS.open(filename, "w");
  if (!file) {
    Serial.println("File open failed");
    return;
  }

  file.println("timestamp_ms,accel_x,accel_y,accel_z");
  Serial.printf("Recording to %s...\n", filename.c_str());

  unsigned long startTime = millis();
  unsigned long interval = 1000 / SAMPLE_RATE_HZ;
  unsigned long nextSample = startTime;

  while (millis() - startTime < DURATION_SEC * 1000) {
    if (millis() >= nextSample) {
      int16_t ax, ay, az;
      mpu.getAcceleration(&ax, &ay, &az);
      unsigned long now = millis();
      file.printf("%lu,%d,%d,%d\n", now - startTime, ax, ay, az);
      nextSample += interval;
    }
  }

  file.close();
  Serial.println("[✔] Recording complete.");
}

void enterDeepSleep() {
  Serial.println("Setting up touchpad wakeup...");

  // Diagnostic print for touch GPIO
  Serial.printf("Touch Wakeup GPIO: %d\n", TOUCH_PAD_NUM);

  // Use the new cross-platform touch wakeup function
  // Threshold value may need adjustment based on your specific hardware
  touchSleepWakeUpEnable(TOUCH_PAD_NUM, 40);  // 40 is a default sensitivity value
  
  // Add timer-based wakeup as a fallback
  esp_sleep_enable_timer_wakeup(30 * 1000000);  // 30 seconds
  
  Serial.println("Configuring touch wakeup...");
  
  Serial.println("Going to deep sleep...");
  delay(100);
  esp_deep_sleep_start();
}

// Optional diagnostic function
void touchPadDiagnostic() {
  Serial.println("Touch Pad Diagnostic:");
  
  // Multiple readings to understand touch sensitivity
  for (int i = 0; i < 10; i++) {
    int touch_value = touchRead(TOUCH_PAD_NUM);
    Serial.printf("Reading %d: %d\n", i, touch_value);
    delay(100);
  }
}

void printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:     Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1:     Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER:    Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP:      Serial.println("Wakeup caused by ULP program"); break;
    default:                        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void printWakeupTouchpad() {
  switch (touchPin) {
    case 0:  Serial.println("Touch detected on GPIO 4"); break;
    case 1:  Serial.println("Touch detected on GPIO 1"); break;
    case 2:  Serial.println("Touch detected on GPIO 2"); break;
    case 3:  Serial.println("Touch detected on GPIO 3"); break;
    case 4:  Serial.println("Touch detected on GPIO 4"); break;
    case 5:  Serial.println("Touch detected on GPIO 5"); break;
    case 6:  Serial.println("Touch detected on GPIO 6"); break;
    case 7:  Serial.println("Touch detected on GPIO 7"); break;
    case 8:  Serial.println("Touch detected on GPIO 8"); break;
    case 9:  Serial.println("Touch detected on GPIO 9"); break;
    default: Serial.println("Wakeup not by touchpad or unknown GPIO"); break;
  }
}
