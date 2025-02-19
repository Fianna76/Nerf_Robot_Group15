#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>

#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#include <OneButton.h>

// ========+++- [Global Variable Setup] -+++========

// --------+++= [MMA8451] =+++--------
Adafruit_MMA8451 mma = Adafruit_MMA8451();

// --------+++= [Servo] =+++--------
const int servoPin = 11;
Servo servo;
int servoPos = 0;

// --------+++= [Joystick] =+++--------
// Pins
const int joystickXPin = A0;
const int joystickYPin = A1;
int joystickButtonPin = 13;

// Deadzone settings
int deadzone1 = 5, deadzone2 = -5, stopVal = 90;
int minSpeed = -60, maxSpeed = 60;
int rotationServoPos, tiltServoPos;

// Joystick values
int joystickXVal, joystickYVal;

// Establish a OneButton object for pushing in the joystick
OneButton btn = OneButton(
    joystickButtonPin,  // Input pin for the button
    true,        // Button is active low
    true         // Enable internal pull-up resistor
  );

// --------+++= [LCD Display] =+++--------

// Define Arduino pins connected to 74LS47 BCD Encoder inputs
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

//Speed (To display on LCD)
int speed = 0;
const long interval = 500;  // Delay speed changes (tickrate)
unsigned long previousMillis = 0;
unsigned long currentMillis;

// ========+++- [Additional Setup Functions] -+++========

//TODO: Can we shift this shit into a custom header file or something?

// Function to setup the joystick - N.B Depends on the global pin variables above
void joystickSetup() {
    pinMode(joystickXPin, INPUT);
    pinMode(joystickYPin, INPUT);
    pinMode(joystickButtonPin, INPUT_PULLUP);
    servo.attach(servoPin);
    servo.write(0);
  }

// MMA8451 Detection and Initialization 
void mmaSetup() {
    if (!mma.begin()) {
        Serial.println("Could not find MMA8451 accelerometer. Check wiring!");
        while (1);
    }

    // Set sensor to 2G range (can be 2, 4, or 8G)
    // TODO: Establish if we need another range (2G should be fine )
    mma.setRange(MMA8451_RANGE_2_G);

    Serial.println("MMA8451 found!");
}

// Setup for BCD *encoder* SN74LS4N - which drives a 5611AH 7 seg LCD
void bcdSetup() {
    // All we need is to set pinmodes
    pinMode(BCD_A, OUTPUT);
    pinMode(BCD_B, OUTPUT);
    pinMode(BCD_C, OUTPUT);
    pinMode(BCD_D, OUTPUT);
}

// ========+++- [Helper Variable Declarations] -+++========

// Updates the LCD display based on angle of controller
void speedChange();

// Handler function for a single click:
void handleClick();

//==============================================SETUP==============================================
void setup() {
    // --------+++= [Serial] =+++--------
    Serial.begin(9600);
    Serial.println("BCD Tilt Intergration Test...");

    // --------+++= [External Setup Functions] =+++--------
    mmaSetup();
    joystickSetup();
    bcdSetup();

    //Built in to OneButton library - attaching our own helper method for when a click is detected
    btn.attachClick(handleClick);  

    }

//==============================================LOOP==============================================

void loop() {
    // --------+++= [Read New Data] =+++--------
    // Read new tilt data
    mma.read();
    // Read joystick values
    joystickXVal = analogRead(joystickXPin);
    joystickYVal = analogRead(joystickYPin);
    //Update button (check for input)
    btn.tick(); 


    // --------+++= [Update LCD] =+++--------
    //Display speed to LCD/Update it based on new mma data
    currentMillis = millis();
    speedChange();

    // --------+++= [Generate Servo Outputs] =+++--------

    servoPos = map(joystickYVal, 0, 1023, 0, 180);

    servo.write(servoPos);   
}

//==========================================HELPER FUNCTIONs==========================================

// Handler function for a single click:
void handleClick()
{
    speed++;
    Serial.println("Button Clicked");
}

void speedChange() {
    // We check the mma values in the y axis to see if the controller is tilted forward or backwards
    // "Forwards" and "Backwards" are hardcoded angles in the first condition of the if statements
    //
    // We also only increment/decrement speed after the time (in milliseconds) set in interval has passed
    // This means the speed change from tilting should act as a single "bump" in a direction, or it can 
    // be held to gradually increase the speed value
    // TODO: Establish another function to use button inputs to quickly set speed to max/min
    
    
    // Detect forward rotation (Increase Speed)
    if((mma.y / 4096.0f) >= 0.5f && currentMillis - previousMillis >= interval && speed < 9)
    {
        speed++;
        previousMillis = currentMillis;
        // Serial.print("Speed up: "); Serial.println(speed);
    }
    // Backwards rotation decreases speed
    else if((mma.y / 4096.0f) <= -0.5f && currentMillis - previousMillis >= interval && speed > 0)
    {
        speed--;
        previousMillis = currentMillis;
        // Serial.print("Speed down: "); Serial.println(speed);
    }

    // Ensure digit is within valid range
    if (speed < 0 || speed > 9) return;

    // Write the BCD bits to the 74LS47 inputs
    digitalWrite(BCD_A, bcdLookup[speed][3]);
    digitalWrite(BCD_B, bcdLookup[speed][2]);
    digitalWrite(BCD_C, bcdLookup[speed][1]);
    digitalWrite(BCD_D, bcdLookup[speed][0]);

}
