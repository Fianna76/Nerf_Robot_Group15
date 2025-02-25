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

#define TILT_INTERVAL (500UL)  // Delay speed changes (tickrate)
unsigned long tiltPreviousMillis = 0;
unsigned long currentMillis;



// --------+++= [Servos] =+++--------
// Pins  
int rotationServoPin = 6;
int tiltServoPin = 8;
int firingServoPin = 11;
int movementServosBackwardsPin = 10;
int movementServoForwardPin = 9;
int brushlessMotor1 = 3;
int brushlessMotor2 = 5;
// int ButtonPin = 4;
// Servo objects
Servo rotationServo, tiltServo, firingServo, movementServosBackwards, movementServoForward;

// Firing Mechanism Values
#define STARTPOS  20
#define FINALPOS 150 
#define FIREDELAY (300UL)
int fireState = 0, shotCount = 0, fireAmount = 0; 
bool isFiring = false;
unsigned long firePreviousMillis = 0;

// Aim Mechanism Values
int rotationServoPos, tiltServoPos;
const unsigned long period = 20000; // 20 ms period in microseconds (50Hz)
const unsigned long minPulse = 1000; // 1 ms pulse (servo minimum position)
const unsigned long maxPulse = 2000; // 2 ms pulse (servo maximum position)

// Movement Mechanism Values
#define BASE_MOVE_INTERVAL (2500UL) //The base amount of time we move for - multiplied by speed
int direction = 0; // Controls direction 
// 0 = Neutral (No movement)
// -1 = CounterClockwise (Left?)
// 1 = Clockwise (Right?)
bool isMoving = false;
unsigned long movePreviousMillis = 0;

// --------+++= [Joystick] =+++--------
// Pins
const int joystickXPin = A0;
const int joystickYPin = A1;
int joystickButtonPin = 13;

// Deadzone settings
int deadzone1 = 10, deadzone2 = -10, stopVal = 90;
int minSpeed = -60, maxSpeed = 60;

// Joystick values
int joystickXVal, joystickYVal;

// Establish a OneButton object for pushing in the joystick
OneButton joystickButton = OneButton(
    joystickButtonPin,  // Input pin for the button
    true,        // Button is active low
    true         // Enable internal pull-up resistor
  );

// --------+++= [LCD Display] =+++--------

// Define Arduino pins connected to 74LS47 BCD Encoder inputs
#define BCD_A 2
#define BCD_B 5
#define BCD_C 4
#define BCD_D 3

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

// ========+++- [Additional Setup Functions] -+++========

//TODO: Can we shift this shit into a custom header file or something?

// Function to setup the joystick - N.B Depends on the global pin variables above
void joystickSetup() {
    pinMode(joystickXPin, INPUT);
    pinMode(joystickYPin, INPUT);
    pinMode(joystickButtonPin, INPUT_PULLUP);
    
    rotationServo.attach(rotationServoPin);
    tiltServo.attach(tiltServoPin);
    firingServo.attach(firingServoPin);
  
    rotationServo.write(0);
    tiltServo.write(0);
    firingServo.write(STARTPOS);
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

    movementServoForward.attach(movementServoForwardPin);
    movementServosBackwards.attach(movementServosBackwardsPin);
    movementServoForward.write(0);
    movementServosBackwards.write(0);
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
  tiltServoPos = map(yVal, 0, 1023, minPulse, maxPulse);

  //make sure the servos dont move when the joystick is in the center position
  if (rotationServoPos > deadzone2 && rotationServoPos < deadzone1) {
    rotationServoPos = stopVal;
  }

  //the servos switch directions on > 90 rather than negative so we add 180 to negative values to get other direction.
  if (rotationServoPos < deadzone2) {
    rotationServo.write(rotationServoPos + 180);
  } else {
    rotationServo.write(rotationServoPos);
  }
  tiltServo.write(tiltServoPos);

  // Debugging output
  // Serial.print("Rotation: "); Serial.print(rotationServoPos);
  // Serial.print(" | Tilt: "); Serial.println(tiltServoPos);
}

void fireLoop() {
  if (!isFiring) return; // Exit if the sequence isn't active

  switch (fireState) {
      case 0: // Move to STARTPOS
          firingServo.write(STARTPOS);
          firePreviousMillis = currentMillis;
          fireState = 1;
          break;

      case 1: // Wait, then move to FINALPOS
          if (currentMillis - firePreviousMillis >= FIREDELAY) {
              firingServo.write(FINALPOS);
              firePreviousMillis = currentMillis;
              fireState = 2;
          }
          break;

      case 2: // Wait, then move back to STARTPOS
          if (currentMillis - firePreviousMillis >= FIREDELAY) {
              firingServo.write(STARTPOS);
              
              shotCount++;

              if (shotCount >= fireAmount) {
                shotCount = 0;
                isFiring = false; // Stop firing after  cycles
            } else {
                fireState = 0; // Restart cycle
          }
          break;
        }
  }
}

void trackLoop() {
  if(!isMoving) return; // Exit when the sequence isn't active 
  
  Serial.print("Direction: "); Serial.println(direction);
  // Currently I have speed implemented to move the robot in a direction for LONGER - at max speed
  // I'll leave it up to you if we should implement it differently - one time period and faster rotation
  // Or change both the length time we move, and the speed at which we move for every speed value

  Serial.println(currentMillis - movePreviousMillis);

  // TODO: Get this to actually work
  switch(direction) {
      case 0:
        // if(currentMillis - movePreviousMillis >= (BASE_MOVE_INTERVAL*(speed+1))) {
          movementServoForward.write(0);
          movementServosBackwards.write(0);
          movePreviousMillis = currentMillis;
        // }
        // else {
          // isMoving = false;
        // }
        break;

      // Move Left (anti-clockwise)
      case -1:
        if(currentMillis - movePreviousMillis >= (BASE_MOVE_INTERVAL)) {
          direction = 0;
        }
        else {
          // N.B. UPDATE THIS TO USE SPEED (* BY IT)
          // also please make sure it works of the 9 gram servo co-ordinates - maybe define a base number we can change?
          movementServoForward.write(30);
          movementServoBackw.write(160);
          // movePreviousMillis = currentMillis;
        }
        break;
      // Move Right (clockwise) 
      case 1:
        if(currentMillis - movePreviousMillis >= (BASE_MOVE_INTERVAL)) {
          direction = 0;
        }
        else {
          // N.B. UPDATE THIS TO USE SPEED (* BY IT)
          // also please make sure it works of the 9 gram servo co-ordinates - maybe define a base number we can change?
          movementServoForward.write(160);
          movementServosBackwards.write(30);
          // movePreviousMillis = currentMillis;
        }
        break;

  }
}

// ========+++- [Helper Function Declarations] -+++========

// Updates the LCD display based on angle of controller
void speedChange();

//Updates the direction to move based on the angle of the controller
void directionChange();

// Handler functions for joystick button:
void handleClick();
void handleLongPressStop();

//==============================================SETUP==============================================

void setup() {
  // --------+++= [Serial] =+++--------
  Serial.begin(9600);
  Serial.println("Intializing Main: ");
  
  // --------+++= [External Setup Functions] =+++--------
  //mmaSetup();
  joystickSetup();
  bcdSetup();
  
  // --------+++= [MISC] =+++--------
  //Built in to OneButton library - attaching our own helper methods for when a click is detected (on the joystick)
  joystickButton.attachClick(handleClick);  
  joystickButton.attachLongPressStop(handleLongPressStop);  
}

//==============================================LOOP==============================================


void loop() {
  // --------+++= [Read New Data] =+++--------
  // Read new tilt data
  //mma.read();
  // Read joystick values
  joystickXVal = analogRead(joystickXPin);
  joystickYVal = analogRead(joystickYPin);
  //Update button (check for input)
  joystickButton.tick(); 


  // --------+++= [Update LCD] =+++--------
  //Display speed to LCD/Update it based on new mma data
  currentMillis = millis();
  speedChange();

  // --------+++= [Generate Servo Outputs] =+++--------
  // Control track movement
  directionChange();
  trackLoop();
  
  // Control turret movement
  joystickLoop(joystickXVal, joystickYVal);
  fireLoop(); // Continuously run fireServo to process the sequence
}

//==========================================HELPER FUNCTIONs==========================================

// Handler function for a single click: Currently Set to fire a single dart
void handleClick() {
  // Serial.println("Joystick Clicked! ");

  if (!isFiring) { 
    isFiring = true; // Start sequence
    fireState = 0; // Ensure we start from the beginning
    fireAmount = 1; //Fire only one bullet
  }
}

// Handler function for a releasing a long press: Currently Set to fire 10 darts 
// TODO: Bullet tracking
void handleLongPressStop() {
  Serial.println("Joystick Released! ");
  if (!isFiring) { 
    isFiring = true; // Start sequence
    fireState = 0; // Ensure we start from the beginning
    fireAmount = 10; //Fire the entire magazine
  }
}

// Changes speed based on angle of controller
void speedChange() {
    // We check the mma values in the y axis to see if the controller is tilted forward or backwards
    // "Forwards" and "Backwards" are hardcoded angles in the first condition of the if fireStatements
    //
    // We also only increment/decrement speed after the time (in milliseconds) set in TILT_INTERVAL has passed
    // This means the speed change from tilting should act as a single "bump" in a direction, or it can 
    // be held to gradually increase the speed value
    // TODO: Establish another function to use button inputs to quickly set speed to max/min
    
    
    // Detect forward rotation (Increase Speed)
    if((mma.y / 4096.0f) >= 0.5f && currentMillis - tiltPreviousMillis >= TILT_INTERVAL && speed < 9)
    {
        speed++;
        tiltPreviousMillis = currentMillis;
        // Serial.print("Speed up: "); Serial.println(speed);
    }
    // Backwards rotation decreases speed
    else if((mma.y / 4096.0f) <= -0.5f && currentMillis - tiltPreviousMillis >= TILT_INTERVAL && speed > 0)
    {
        speed--;
        tiltPreviousMillis = currentMillis;
        // Serial.print("Speed down: "); Serial.println(speed);
    }

    // Ensure digit is within valid range
    if (speed < 0 || speed > 9) return;

    // Write the BCD bits to the 74LS47 inputs
    digitalWrite(BCD_A, bcdLookup[speed][3]);
    digitalWrite(BCD_B, bcdLookup[speed][2]);
    digitalWrite(BCD_C, bcdLookup[speed][1]);
    digitalWrite(BCD_D, bcdLookup[speed][0]);

    // Serial.println(speed);
}

//Changes movement direction based on angle of controller
void directionChange() {
  // Right
  if((mma.x / 4096.0f) >= 0.25f && currentMillis - tiltPreviousMillis >= TILT_INTERVAL) {
    direction = 1;
    isMoving = true;
    // Serial.println("Moving Right");
  }
  // Left
  else if((mma.x / 4096.0f) <= -0.25f && currentMillis - tiltPreviousMillis >= TILT_INTERVAL)
  {
    direction = -1;
    isMoving = true;
    // Serial.println("Moving Left");
  }
}