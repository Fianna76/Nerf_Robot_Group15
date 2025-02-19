#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>

#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#include <OneButton.h>

//TODO: I'm factoring this code to work off my UNO cause that's how my IDE is configured right now
//      Need to change the pinouts to work with the NANO 

// ========+++- [Global Variable Setup] -+++========

// --------+++= [MMA8451] =+++--------
Adafruit_MMA8451 mma = Adafruit_MMA8451();
// Note: For Uno
// SCL = A5
// SDA = A4 

// --------+++= [Servos] =+++--------

// TODO: These pins *conflict* with the pins I currently have set to our BCD->LCD encoder - change em

// Pins  
int rotationServoPin = 9;
int tiltServoPin = 8;
int firingServoPin = 10;
// int ButtonPin = 4;
// Servo objects
Servo rotationServo, tiltServo, firingServo;

// Position Values
int startPos = 90, angleRange = 10, delayTime = 5; // Firing Mechanism
int rotationServoPos, tiltServoPos;

// --------+++= [Joystick] =+++--------
// Pins
const int joystickXPin = A0;
const int joystickYPin = A1;
int joystickButtonPin = 13;

// Deadzone settings
int deadzone1 = 5, deadzone2 = -5, stopVal = 90;
int minSpeed = -60, maxSpeed = 60;

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
    
    rotationServo.attach(rotationServoPin);
    tiltServo.attach(tiltServoPin);
    
    rotationServo.write(0);
    tiltServo.write(0);
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

// ========+++- [Additional Loop Functions] -+++========

// Function to control the turret's movement
void joystickLoop(int xVal, int yVal) {
  //0 is still, 90 max speed clockwise, 180 max speed counter-clockwise
  rotationServoPos = map(xVal, 0, 1023, minSpeed, maxSpeed);
  tiltServoPos = map(yVal, 0, 1023, minSpeed, maxSpeed);

  //make sure the servos dont move when the joystick is in the center position
  if (rotationServoPos > deadzone2 && rotationServoPos < deadzone1) {
    rotationServoPos = stopVal;
  }
  if (tiltServoPos > deadzone2 && tiltServoPos < deadzone1) {
    tiltServoPos = stopVal;
  }

  //the servos switch directions on > 90 rather than negative so we add 180 to negative values to get other direction.
  if (rotationServoPos < deadzone2) {
    rotationServo.write(rotationServoPos + 180);
  } else {
    rotationServo.write(rotationServoPos);
  }
  if (tiltServoPos < deadzone2) {
    tiltServo.write(tiltServoPos + 180);
  } else {
    tiltServo.write(tiltServoPos);
  }

  // Debugging output
  // Serial.print("Rotation: "); Serial.print(rotationServoPos);
  // Serial.print(" | Tilt: "); Serial.println(tiltServoPos);
}

// Function to handle firing mechanism
// TODO: Refactor this 
//    - don't use delay (Seizes up entire robot)
//    - use the joystick button - this function is now called in the handleClick helper method

void firingLoop() {
    for (int pos = 80; pos <= 100; pos++) {
      firingServo.write(pos);
      delay(delayTime);
    }
    for (int pos = 100; pos >= 80; pos--) {
      firingServo.write(pos);
      delay(delayTime);
    }
  firingServo.write(90); 
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
  Serial.println("Intializing Main: ");
  
  // --------+++= [External Setup Functions] =+++--------
  mmaSetup();
  joystickSetup();
  bcdSetup();

  //Built in to OneButton library - attaching our own helper method for when a click is detected
  btn.attachClick(handleClick);  
  
  // --------+++= [MISC] =+++--------

  //TODO: Take this out of setup?
  firingServo.attach(firingServoPin);
  firingServo.write(90);
  
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

  // Control turret movement
  joystickLoop(joystickXVal, joystickYVal);

}

//==========================================HELPER FUNCTIONs==========================================

// Handler function for a single click:
// Currently Set to fire a single dart
void handleClick()
{
    firingLoop();
    Serial.println("Button Clicked");
}

// Changes speed based on angle of controller
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