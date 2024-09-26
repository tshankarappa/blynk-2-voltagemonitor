#include <Arduino.h>
#include <math.h>

#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL3ulSDD24e"
#define BLYNK_TEMPLATE_NAME "Test1"
#define BLYNK_AUTH_TOKEN "6Ef2CzHO7Xm8C6woP4AZ5KHVb9ubc7qI"
#include <BlynkSimpleEsp32.h>

#define ZMPT101B_PIN 34  // GPIO pin for reading the ZMPT101B sensor

const unsigned long updateInterval = 2000; // Send data every 2 seconds
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
const float scalingFactor = 70; // Scaling factor for 220V

void setup() {
  Serial.begin(115200);
  pinMode(ZMPT101B_PIN, INPUT); // Set ZMPT101B pin as input
  Blynk.begin(BLYNK_AUTH_TOKEN, "Guest", "asdfghjkl"); 
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

void loop() {
  // Read the value from the ZMPT101B sensor
  readings[currentIndex] = analogRead(ZMPT101B_PIN);
  currentIndex = (currentIndex + 1) % numReadings;
  
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= updateInterval) { // Every 2 seconds
    float rmsVoltage = calculateRMS();
    float acVoltage = calculateACVoltage(rmsVoltage);
    
    /* Print results to serial monitor
    Serial.print("RMS Voltage: ");
    Serial.println(rmsVoltage);
    Serial.print("Peak Voltage: ");
    Serial.println(rmsVoltage * sqrt(2));
    Serial.print("Peak-to-Peak Voltage: ");
    Serial.println(2 * rmsVoltage * sqrt(2));
    Serial.print("Actual AC Voltage (220V scaled): ");
    Serial.println(acVoltage);
    */
    // Send the voltage to Blynk
    Blynk.virtualWrite(V4, acVoltage);  
    
    lastPrintTime = currentTime;
  }
  
  // Delay to achieve approximately 150 readings per second
  delay(6); // 1000 ms / 150 readings = ~6.67 ms per reading
}
