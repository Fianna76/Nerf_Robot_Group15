#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_I2CDevice.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#include <OneButton.h>

// Define Arduino pins connected to 74LS47 BCD inputs
const int BCD_A = 2;
const int BCD_B = 5;
const int BCD_C = 4;
const int BCD_D = 3;

// Lookup table for BCD representations of digits 0–9
const byte bcdLookup[10][4] = {
  {0, 0, 0, 0}, // 0
  {0, 0, 0, 1}, // 1
  {0, 0, 1, 0}, // 2
  {0, 0, 1, 1}, // 3
  {0, 1, 0, 0}, // 4
  {0, 1, 0, 1}, // 5
  {0, 1, 1, 0}, // 6
  {0, 1, 1, 1}, // 7
  {1, 0, 0, 0}, // 8
  {1, 0, 0, 1}  // 9
};

//Actual Output
//0 -> 2

void displayDigit(int digit);

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  Serial.println("Starting 7-Segment Display Test...");

  // Set BCD pins as outputs
  pinMode(BCD_A, OUTPUT);
  pinMode(BCD_B, OUTPUT);
  pinMode(BCD_C, OUTPUT);
  pinMode(BCD_D, OUTPUT);
}

void loop() {
  for (int digit = 0; digit < 10; digit++) {
    displayDigit(digit);
    delay(1000); // Wait for 1 second
  }
  
}

void displayDigit(int digit) {
  // Ensure digit is within valid range
  if (digit < 0 || digit > 9) return;

   // Write the BCD bits to the 74LS47 inputs
   digitalWrite(BCD_A, bcdLookup[digit][3]);
   digitalWrite(BCD_B, bcdLookup[digit][2]);
   digitalWrite(BCD_C, bcdLookup[digit][1]);
   digitalWrite(BCD_D, bcdLookup[digit][0]);

  // Debugging output
  Serial.print("Displaying digit: ");
  Serial.println(digit);

  // Serial.println("A: " + BCD_A);
  // Serial.println("B: " + BCD_B);
  // Serial.println("C: " + BCD_C);
  // Serial.println("D: " + BCD_D);
}
