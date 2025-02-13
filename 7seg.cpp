#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

//Need to include these because the libraries are included and it wont compile otherwise
#include <Adafruit_I2CDevice.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

// Define function prototype
void displayDigit(int digit);

// Define the Arduino pins connected to the 5611AH segments
const int segmentPins[7] = {2, 3, 4, 5, 6, 7, 8}; // A, B, C, D, E, F, G


// Digit patterns for 0-9 (common cathode)
const byte digitPatterns[10] = {
  B0111111, // 0
  B0000110, // 1
  B1011011, // 2
  B1001111, // 3
  B1100110, // 4
  B1101101, // 5
  B1111101, // 6
  B0000111, // 7
  B1111111, // 8
  B1101111  // 9
};

void setup() {
  Serial.begin(9600); // Initialize Serial Monitor
  Serial.println("Starting Seven Segment Display Test...");

  // Set all segment pins as outputs
  for (int i = 0; i < 7; i++) {
    pinMode(segmentPins[i], OUTPUT);
    Serial.print("Setting pin ");
    Serial.print(segmentPins[i]);
    Serial.println(" as OUTPUT.");
  }
}

void loop() {
  for (int digit = 0; digit < 10; digit++) {
    Serial.print("Displaying digit: ");
    Serial.println(digit);
    displayDigit(digit);
    delay(1000); // Wait for 1 second
  }
}

// Function to display a digit
void displayDigit(int digit) {
  Serial.print("Setting segments for digit: ");
  Serial.println(digit);

  byte segments = digitPatterns[digit];

  for (int i = 0; i < 7; i++) {
    bool state = bitRead(segments, i);
    digitalWrite(segmentPins[i], state);
    
    Serial.print("Pin ");
    Serial.print(segmentPins[i]);
    Serial.print(" -> ");
    Serial.println(state ? "HIGH (ON)" : "LOW (OFF)");
  }
}