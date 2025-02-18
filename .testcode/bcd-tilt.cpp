#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_I2CDevice.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#include <OneButton.h>

// Create an MMA8451 instance
Adafruit_MMA8451 mma = Adafruit_MMA8451();

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

//Speed display
int speed = 0;
const long interval = 500;  // Delay speed changes
unsigned long previousMillis = 0;

void displayDigit(int digit);

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(9600);
    Serial.println("BCD Tilt Intergration Test...");

    // Initialize MMA8451
    if (!mma.begin()) {
        Serial.println("Could not find MMA8451 accelerometer. Check wiring!");
        while (1);
    }

    Serial.println("MMA8451 found!");

    // Set sensor to 2G range (can be 2, 4, or 8G)
    mma.setRange(MMA8451_RANGE_2_G);

        // Set BCD pins as outputs
        pinMode(BCD_A, OUTPUT);
        pinMode(BCD_B, OUTPUT);
        pinMode(BCD_C, OUTPUT);
        pinMode(BCD_D, OUTPUT);
    }

void loop() {
    // Read new data
   mma.read();
    
   // Get acceleration values (X, Y, Z)
   Serial.print("X: "); Serial.print(mma.x / 4096.0); Serial.print(" m/s²");
   Serial.print(" | Y: "); Serial.print(mma.y / 4096.0); Serial.print(" m/s²");
   Serial.print(" | Z: "); Serial.print(mma.z / 4096.0); Serial.println(" m/s²");
   
   //Speed display change
   unsigned long currentMillis = millis();
   
   if((mma.y / 4096.0) >= 0.5f && currentMillis - previousMillis >= interval && speed < 9)
   {
        speed++;
        previousMillis = currentMillis;
   }
   else if((mma.y / 4096.0) <= -0.5f && currentMillis - previousMillis >= interval && speed>0)
   {
        speed--;
        previousMillis = currentMillis;
   }

   delay(500); // Wait 500ms before the next reading 
   displayDigit(speed);
  
}

void displayDigit(int digit) {
  // Ensure digit is within valid range
  if (digit < 0 || digit > 9) return;

   // Write the BCD bits to the 74LS47 inputs
   digitalWrite(BCD_A, bcdLookup[digit][3]);
   digitalWrite(BCD_B, bcdLookup[digit][2]);
   digitalWrite(BCD_C, bcdLookup[digit][1]);
   digitalWrite(BCD_D, bcdLookup[digit][0]);
}
