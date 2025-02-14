#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_I2CDevice.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#include <OneButton.h>

// Define Arduino pins connected to 74LS47 BCD inputs
const int BCD_A = 2;
const int BCD_B = 4;
const int BCD_C = 5;
const int BCD_D = 3;

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

  // Display the digit
  digitalWrite(BCD_A, bitRead(digit, 0));
  digitalWrite(BCD_B, bitRead(digit, 1));
  digitalWrite(BCD_C, bitRead(digit, 2));
  digitalWrite(BCD_D, bitRead(digit, 3));

  // Debugging output
  Serial.print("Displaying digit: ");
  Serial.println(digit);
}
