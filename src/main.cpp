#include <Arduino.h>
#include <math.h>

#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL3ulSDD24e"
#define BLYNK_TEMPLATE_NAME "Test1"
#define BLYNK_AUTH_TOKEN "6Ef2CzHO7Xm8C6woP4AZ5KHVb9ubc7qI"
#include <BlynkSimpleEsp32.h>

#define ZMPT101B_PIN 34  // GPIO pin for reading the ZMPT101B sensor
// Wi-Fi credentials
char ssid[] = "Guest";
char pass[] = "asdfghjkl";

const unsigned long updateInterval = 30000; // Send data every 30 seconds
const int numReadings = 150; // Number of readings per second
int readings[numReadings];   // Array to store readings
int currentIndex = 0;        // Current index in the array
unsigned long lastPrintTime = 0;
const int offsetValue = 2948; // Approximate offset when power is off
const float voltageReference = 5.0; // ADC reference voltage
const int adcMaxValue = 4095; // Maximum ADC value

// Calibration constants
const float knownRMSVoltage = 220.0; // Known RMS voltage
const float calibrationRMSVoltage = 1.2; // RMS voltage measured with known 220V
const float scalingFactor = 64; // Scaling factor for 220V


unsigned long lastReconnectAttempt = 0; // Keep track of last reconnection attempt
const unsigned long reconnectInterval = 30000; // Retry every 10 seconds

void setup() {
  Serial.begin(115200);
  pinMode(ZMPT101B_PIN, INPUT); // Set ZMPT101B pin as input

  // Connect to Wi-Fi manually
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // Configure Blynk without blocking
  Blynk.config(BLYNK_AUTH_TOKEN);
}

float calculateRMS() {
  float sumOfSquares = 0.0;
  int validCount = 0;

  for (int i = 0; i < numReadings; ++i) {
    int value = readings[i];
    // For AC, use the offset to adjust for zero crossing
    sumOfSquares += pow(value - offsetValue, 2);
    validCount++;
  }

  float rmsVoltage = validCount > 0 ? sqrt(sumOfSquares / validCount) * (voltageReference / adcMaxValue) : 0;
  return rmsVoltage;
}

float calculateACVoltage(float rmsVoltage) {
  float peakVoltage = rmsVoltage * sqrt(2);
  float peakToPeakVoltage = 2 * peakVoltage;
  return peakToPeakVoltage * scalingFactor;
}

void reconnectBlynk() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastReconnectAttempt >= reconnectInterval) {
    lastReconnectAttempt = currentMillis;  // Update last attempt time
    if (!Blynk.connected()) {
      Serial.println("Attempting to reconnect to Blynk...");
      Blynk.connect(2000); // Attempt to reconnect for 2 seconds
    }
  }
}

void loop() {
  // Ensure Wi-Fi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected, reconnecting...");
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWi-Fi reconnected!");
  }

  // Handle Blynk reconnection if needed
    if (WiFi.status() == WL_CONNECTED && !Blynk.connected())
    {
      reconnectBlynk();
    }

  // Read the value from the ZMPT101B sensor
  readings[currentIndex] = analogRead(ZMPT101B_PIN);
  currentIndex = (currentIndex + 1) % numReadings;

  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= updateInterval) {
    float rmsVoltage = calculateRMS();
    float acVoltage = calculateACVoltage(rmsVoltage);

    if (Blynk.connected()) {
      Blynk.virtualWrite(V4, acVoltage);
    } else {
      Serial.println("Blynk not connected, skipping data send");
    }

    lastPrintTime = currentTime;
    delay(updateInterval - 1000);
  }

  // Ensure Blynk tasks are handled
  Blynk.run();

  // Delay to achieve approximately 150 readings per second
  delay(6);
}
