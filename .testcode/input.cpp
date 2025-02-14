#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_I2CDevice.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#include <OneButton.h>


// Create an MMA8451 instance
Adafruit_MMA8451 mma = Adafruit_MMA8451();

// mma pins
//const int SCL = A5; // Serial Clock
//const int SDA = A4; // Serial Data

// Joystick pins
#define joystickXPin A0
#define joystickYPin A1
#define joystickButtonPin 13

OneButton btn = OneButton(
    joystickButtonPin,  // Input pin for the button
    true,        // Button is active low
    true         // Enable internal pull-up resistor
  );

// Handler function for a single click:
void handleClick();

// Deadzone settings
int deadzone1 = 5, deadzone2 = -5, stopVal = 90;
int minSpeed = -60, maxSpeed = 60;
int rotationServoPos, tiltServoPos;

// Joystick values
int joystickXVal, joystickYVal;

// Function to setup the joystick
void joystickSetup() {
    pinMode(joystickXPin, INPUT);
    pinMode(joystickYPin, INPUT);
    pinMode(joystickButtonPin, INPUT_PULLUP);
  }
  

// Define the Arduino pins connected to the 5611AH segments
const int segmentPins[4] = {2, 3, 4, 5}; // A, B, C, D

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

// Define function prototype
void speedDisplay(unsigned long currentMillis);

//Speed display
int speed = 0;
const long interval = 500;  // Delay speed changes
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(9600); // Initialize Serial Monitor
  Serial.println("Starting Input Test...");

  // Setup joystick and servos
  joystickSetup();
  

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

    btn.attachClick(handleClick);
  }
}

void loop() {
    // Read new data
    mma.read();

    // Get acceleration values (X, Y, Z)
    // Serial.print("\nX: "); Serial.print(mma.x / 4096.0); Serial.print(" m/s²");
    // Serial.print(" | Y: "); Serial.print(mma.y / 4096.0); Serial.print(" m/s²");
    // Serial.print(" | Z: "); Serial.print(mma.z / 4096.0); Serial.println(" m/s²");

    //Speed display change
    speedDisplay(millis()); 

    // Read joystick values
    joystickXVal = analogRead(joystickXPin);
    joystickYVal = analogRead(joystickYPin);
    
    Serial.print("\nJoystick X: "); Serial.print(joystickXVal);
    Serial.print(" | Y: "); Serial.print(joystickYVal);

    btn.tick();    

    //delay(500); // Wait 500ms before the next reading 
}

// Function to display a digit
void speedDisplay(unsigned long currentMillis) {
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
  
//   Serial.print("Setting segments for speed: ");
//   Serial.println(speed);

  byte segments = digitPatterns[speed];

  for (int i = 0; i < 7; i++) {
    bool state = bitRead(segments, i);
    digitalWrite(segmentPins[i], state);
    
    // Serial.print("Pin ");
    // Serial.print(segmentPins[i]);
    // Serial.print(" -> ");
    // Serial.println(state ? "HIGH (ON)" : "LOW (OFF)");
  }
}

// Handler function for a single click:
void handleClick()
{
    Serial.println("Clicked!");
    delay(2000);
}