#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_I2CDevice.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

// Create an MMA8451 instance
Adafruit_MMA8451 mma = Adafruit_MMA8451();

// mma pins
//const int SCL = A5; // Serial Clock
//const int SDA = A4; // Serial Data

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

//Speed display
int speed = 0;
const long interval = 500;  // Delay speed changes
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(9600); // Initialize Serial Monitor
  Serial.println("Starting Seven Segment Display Test...");

  // Set all segment pins as outputs
  for (int i = 0; i < 7; i++) {
    pinMode(segmentPins[i], OUTPUT);
    Serial.print("Setting pin ");
    Serial.print(segmentPins[i]);
    Serial.println(" as OUTPUT.");

    // Initialize MMA8451
    if (!mma.begin()) {
        Serial.println("Could not find MMA8451 accelerometer. Check wiring!");
        while (1);
    }
    
    Serial.println("MMA8451 found!");
    
    // Set sensor to 2G range (can be 2, 4, or 8G)
    mma.setRange(MMA8451_RANGE_2_G);
  }
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

// Function to display a digit
void displayDigit(int digit) {
  Serial.print("Setting segments for digit: ");
  Serial.println(digit);

  byte segments = digitPatterns[digit];

  for (int i = 0; i < 7; i++) {
    bool state = bitRead(segments, i);
    digitalWrite(segmentPins[i], state);
    
    // Serial.print("Pin ");
    // Serial.print(segmentPins[i]);
    // Serial.print(" -> ");
    // Serial.println(state ? "HIGH (ON)" : "LOW (OFF)");
  }
}