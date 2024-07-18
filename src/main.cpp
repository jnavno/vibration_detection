#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "SPIFFS.h"

#define INTERRUPT_PIN GPIO_NUM_7
#define LED_PIN 17
#define INSPECTION_DURATION_SECONDS 11  // sampling duration time in seconds

#define MAX_SAMPLES_PER_CYCLE 100               // Maximum number of samples per sampling cycle
#define CSV_FILENAME "/accelerometer_data.csv"  // CSV file name

// Threshold values for vibration sensitivity
// Adjust these values to change the sensitivity
#define THRESHOLD_TIER1 210  // lower threshold
#define THRESHOLD_TIER2 225  // higherthreshold

volatile bool triggered = false;

RTC_DATA_ATTR int bootCount = 0;

float weight = 0.102;    // Combined weight of the object in kg (102 grams converted to kilograms)
float totalWorkX = 0.0;  // Total work done on X axis
float totalWorkY = 0.0;  // Total work done on Y axis
float totalWorkZ = 0.0;  // Total work done on Z axis
float totalWork = 0.0;   // Total work done on all axes

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void handleInterrupt() {
  triggered = true;
}

// Structure to hold accelerometer data
typedef struct {
  float acceleration[3];    // Acceleration values for each axis
  float work[3];            //work values for each axis
  unsigned long timestamp;  // Timestamp of the sample
} AccelerometerSample;

bool inspectTreeShaking() {
  unsigned long startTime = millis();
  unsigned long elapsedTime = 0;
  int sample_count = 0;

  // Defining the array to store accelerometer samples
  AccelerometerSample samples[MAX_SAMPLES_PER_CYCLE];

  Adafruit_MPU6050 mpu;
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    return false;
  }

  Serial.println("Checking for tree shaking...");

  // Open the CSV file for writing
  File file = SPIFFS.open(CSV_FILENAME, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return false;
  }

  // Write CSV header
  file.println("Timestamp,X_Acceleration,Y_Acceleration,Z_Acceleration,Work_X,Work_Y,Work_Z,Total_Work");

  while (elapsedTime < INSPECTION_DURATION_SECONDS * 1000 && sample_count < MAX_SAMPLES_PER_CYCLE) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Compute work for each axis
    float workX = weight * fabs(a.acceleration.x);
    float workY = weight * fabs(a.acceleration.y);
    float workZ = weight * fabs(a.acceleration.z);


    // Compute total work
    totalWorkX += workX;
    totalWorkY += workY;
    totalWorkZ += workZ;
    totalWork += workX + workY + workZ;


    // Store accelerometer data and work values in the samples array
    samples[sample_count].acceleration[0] = a.acceleration.x;
    samples[sample_count].acceleration[1] = a.acceleration.y;
    samples[sample_count].acceleration[2] = a.acceleration.z;
    samples[sample_count].work[0] = workX;
    samples[sample_count].work[1] = workY;
    samples[sample_count].work[2] = workZ;
    samples[sample_count].timestamp = millis();

    // Write accelerometer data and work values to the CSV file
    file.print(samples[sample_count].timestamp);
    file.print(",");
    file.print(samples[sample_count].acceleration[0]);
    file.print(",");
    file.print(samples[sample_count].acceleration[1]);
    file.print(",");
    file.print(samples[sample_count].acceleration[2]);
    file.print(",");
    file.print(samples[sample_count].work[0]);
    file.print(",");
    file.print(samples[sample_count].work[1]);
    file.print(",");
    file.print(samples[sample_count].work[2]);
    file.print(",");
    file.println(totalWork);

    // Printing accelerometer data to Serial
    Serial.printf("Accel X: %.2f, Y: %.2f, Z: %.2f\n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
    Serial.printf("Work X: %.2f, Y: %.2f, Z: %.2f\n", workX, workY, workZ);

    // Increment sample count
    sample_count++;

    // Update elapsed time
    elapsedTime = millis() - startTime;
    delay(100);  // Wait for 100ms between readings
  }

  //Close the CSV file
  file.close();

  // Print the total work done on each axis and total work done on all axes
  Serial.println("Total work done on X axis: " + String(totalWorkX));
  Serial.println("Total work done on Y axis: " + String(totalWorkY));
  Serial.println("Total work done on Z axis: " + String(totalWorkZ));
  Serial.println("Total work done on all axes: " + String(totalWork));

  //TODO think about a samplig behaviour for testing only and define it here
  // like taking samples as a single csv file only from wake up to deep sleep

  // Checking if enough samples were collected
  if (sample_count < MAX_SAMPLES_PER_CYCLE) {
    Serial.println("Not enough samples collected. Exiting.");
    return false;
  }

  return true;
}

void sendAlarm() {
  // Blink LED to indicate alarm is activated
  for (int i = 0; i < 12; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  Serial.println("Alarm sent: Strong Tree shaking detected. Potential thread at x,y coordinates");
}

void toggleAccelPower(bool state) {
  if (state) {
    Serial.println("Turning on accelerometer...");
    // Code to toggle accelerometer into power ON
    delay(2000);  // Delay to allow the accelerometer to stabilize
  } else {
    Serial.println("Turning off accelerometer...");
    // Code to toggle accelerometer into power OFF
    delay(2000);  // Delay to allow the accelerometer to power down
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Take some time to open up the Serial Monitor

  // Increment boot number and print it every reboot to indicate what cycle we are in
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  // Print the wakeup reason for ESP32
  print_wakeup_reason();

  // Configure LED pin
  pinMode(LED_PIN, OUTPUT);

  // Attach interrupt to pin 7
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, RISING);

  // Enable wake-up from pin 7 (EXT0) when it goes high
  esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, HIGH);

  // Configure I2C pins (SDA - GPIO 10, SCL - GPIO 11)
  Wire.begin(10, 11);

  // Mount SPIFFS filesystem
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS filesystem");
    return;
  }

  // Check for accelerometer chip
  Adafruit_MPU6050 mpu;
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    return;
  }
  Serial.println("MPU6050 chip detected");

  bool tooMuchShake = false;
  int tooMuchShakeCount = 0;
  int consecutiveNoShakeCount = 0;

  // Perform sampling cycles, at this point 5
  for (int i = 0; i < 5; i++) {
    Serial.print("Starting sampling cycle ");
    Serial.println(i + 1);

    // Turn on accelerometer power
    toggleAccelPower(true);

    // Wait for 11 seconds before performing sampling
    delay(5000);

    // Perform the first sampling cycle
    Serial.println("Starting first sampling cycle");

    // Perform sampling
    totalWorkX = 0.0;
    totalWorkY = 0.0;
    totalWorkZ = 0.0;
    bool tooMuchShakeResult = inspectTreeShaking();
    toggleAccelPower(false);

    Serial.print("Sampling cycle ");
    Serial.print(i + 1);
    Serial.println(" completed");

    // Display status of detection variables after each sampling cycle
    Serial.print("Too much shake detected: ");
    Serial.println(tooMuchShake ? "Yes" : "No");

    Serial.print("Too much shake count: ");
    Serial.println(tooMuchShakeCount);

    Serial.print("Consecutive no shake count: ");
    Serial.println(consecutiveNoShakeCount);

    // Display grand totals after each sampling cycle
    Serial.println("Grand totals after sampling cycle " + String(i + 1) + ":");
    Serial.println("Total work done on all axes: " + String(totalWork));
    Serial.println();

    // Check if not enough shaking was detected after the first sampling cycle
    if (i == 0 && totalWork < THRESHOLD_TIER1) {
      // If total work done is less than the threshold after the first sampling cycle, go back to sleep
      Serial.println("Not enough shake detected after the first sampling cycle, going back to deep sleep");
      esp_deep_sleep_start();
    }

    // Check if threshold is surpassed
    if (totalWork > THRESHOLD_TIER2) {
      tooMuchShake = true;
    } else if (totalWork > THRESHOLD_TIER1) {
      tooMuchShakeCount++;
      consecutiveNoShakeCount = 0;  // Reset consecutive no shake count
    } else {
      tooMuchShakeCount = 0;  // Reset too much shake count
      consecutiveNoShakeCount++;
    }

    // Check if alarm should be sent
    if (tooMuchShakeCount >= 3) {
      // Send alarm
      sendAlarm();
      break;  // Exit loop if alarm is sent
    } else if (consecutiveNoShakeCount >= 3 || (i == 4 && tooMuchShakeCount <= 2)) {
      // Not enough shake or acceptable number of shakes, go back to sleep
      Serial.println("Not enough or acceptable shake detected, going back to deep sleep");
      esp_deep_sleep_start();
    }

    // If not last cycle, go back to sleep
    if (i < 4) {
      Serial.println("Waiting for next sampling cycle...");
      delay(2000);  // Delay before starting next cycle
    }
  }

  // Go to sleep
  Serial.println("Going to sleep zzz");
  esp_deep_sleep_start();
}
void loop() {
  // This is not going to be called
}
