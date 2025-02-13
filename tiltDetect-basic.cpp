#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

// Create an MMA8451 instance
Adafruit_MMA8451 mma = Adafruit_MMA8451();

void setup() {
    Serial.begin(9600);
    
    // Initialize MMA8451
    if (!mma.begin()) {
        Serial.println("Could not find MMA8451 accelerometer. Check wiring!");
        while (1);
    }
    
    Serial.println("MMA8451 found!");
    
    // Set sensor to 2G range (can be 2, 4, or 8G)
    mma.setRange(MMA8451_RANGE_2_G);
}

void loop() {
    // Read new data
    mma.read();
    
    // Get acceleration values (X, Y, Z)
    Serial.print("X: "); Serial.print(mma.x / 4096.0); Serial.print(" m/s²");
    Serial.print(" | Y: "); Serial.print(mma.y / 4096.0); Serial.print(" m/s²");
    Serial.print(" | Z: "); Serial.print(mma.z / 4096.0); Serial.println(" m/s²");
    
    delay(500); // Wait 500ms before the next reading
}