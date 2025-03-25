#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <OneButton.h>

// ========+++- [Global Variable Setup] -+++========

// --------+++= [MMA8451] =+++--------
// Initialize MMA8451 accelerometer at address 0x1D
Adafruit_MMA8451 mma = Adafruit_MMA8451(0x1D);

#define TILT_INTERVAL (500UL)  // Delay speed changes (tickrate)
unsigned long tiltPreviousMillis = 0;
unsigned long currentMillis;

// --------+++= [Servos] =+++--------

// Servo Objects
Servo yawServo;
Servo pitchServo;
Servo feedServo;
Servo flywheelServo;
Servo frontWheel;
Servo backWheel;

// Pin definitions
const int yawPin = 9;
const int pitchPin = 10;
const int feedPin = 6;
const int flywheelPin = 5;
const int frontWheelPin = 11;  
const int backWheelPin = 12; 

boolean flywheelEnabled = false;

// --------+++= [Analogue Inputs] =+++--------
const int joystickX = A0; // Yaw control
const int joystickY = A1; // Pitch control

const int potPin = A2; //Potentiometer 


// --------+++= [D-Pad] =+++--------
const int dpadUpPin = 4;
const int dpadDownPin = 7;
const int dpadLeftPin = 2;
const int dpadRightPin = 3;

OneButton dpadUp(dpadUpPin, true); //True for Active LOW pull-up
OneButton dpadDown(dpadDownPin, true);
OneButton dpadLeft(dpadLeftPin, true);
OneButton dpadRight(dpadRightPin, true);

// --------+++= [LCD Display] =+++--------

// Define Arduino pins connected to 74LS47 BCD Encoder inputs
#define BCD_A 13
#define BCD_B 1
#define BCD_C 0
#define BCD_D 8

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

// --------+++= [Global Variables] =+++--------
// Aiming
int yawMapped = 0;
int pitchMapped = 0;
int yawSpeed = 90;      // Centered at 90 (neutral)
int pitchSpeed = 90;    // Centered at 90 (neutral)

// Firing
int flywheelSpeed = 0;
int feedPosition = 25;  // Default position

// Movment
int leftWheelSpeed = 90;
int rightWheelSpeed = 90;
int speed = 0; // 7Seg Display

// ========+++- [Helper Function Declarations] -+++========

// Changes speed based on angle of controller
void speedChange();

// Stops all motion on button press
void stop();

// Toggles flywheel input from potentiometer (on button press)
void flywheelToggle();

// Toggles between Micro/Macro adjustments of aim with joystick (on button press)
void joystickToggle();

// Fires a single bullet on button press
void fire();

//==============================================SETUP==============================================
void setup() {
    Serial.begin(4800); //TODO: At 4600 only for testing
    Serial.println("Initializing system...");

    // Attach servos
    yawServo.attach(yawPin);
    pitchServo.attach(pitchPin);
    feedServo.attach(feedPin);
    flywheelServo.attach(flywheelPin);
    frontWheel.attach(frontWheelPin);
    backWheel.attach(backWheelPin);

    // Initialize servos to neutral position
    yawServo.write(90);
    pitchServo.write(90);
    feedServo.write(90);
    flywheelServo.write(0);
    frontWheel.write(90);
    backWheel.write(90);

    Serial.println("Servos initialized..."); 

    // Initialize Buttons
    pinMode(dpadUp, INPUT_PULLUP);
    dpadUp.attachClick(stop()); //Single Click 

    pinMode(dpadDown, INPUT_PULLUP);
    dpadDown.attachClick(flywheelToggle);

    pinMode(dpadLeft, INPUT_PULLUP);
    dpadLeft.attachClick(fire()); //TODO: Determine something for this/move fire integration

    pinMode(dpadRight, INPUT_PULLUP);
    dpadRight.attachClick(joystickToggle);

    Serial.println("Buttons initialized..."); 

    // Initialize BCD Encoder
    pinMode(BCD_A, OUTPUT);
    pinMode(BCD_B, OUTPUT);
    pinMode(BCD_C, OUTPUT);
    pinMode(BCD_D, OUTPUT);

    speed = 0;

    Serial.println("7Seg initialized..."); 

    // Initialize MMA8451 accelerometer
    if (!mma.begin()) {
        Serial.println("MMA8451 not found! Check wiring.");
        while (1);
    }
    Serial.println("MMA8451 detected successfully!");

    // Set MMA8451 sensitivity (2G range for better precision)
    mma.setRange(MMA8451_RANGE_2_G);
}

void loop() {
    // --------+++= [Read New Data] =+++--------
    // Read new tilt data
    mma.read();

    // Read joystick values
    int joyX = analogRead(joystickX);
    int joyY = analogRead(joystickY);
    // Map joystick values
    yawSpeed = map(joyX, 0, 1023, 0, 180); 
    pitchSpeed = map(joyY, 0, 1023, 0, 180); 

    // Read potentiometer for flywheel speed & map values
    int potValue = analogRead(potPin);
    flywheelSpeed = map(potValue, 0, 1023, 30, 165);

    // Read buttons to see if there's an input
    dpadUp.tick();
    dpadDown.tick();
    dpadLeft.tick();
    dpadRight.tick();
    
    // --------+++= [Update LCD] =+++--------
    //Display speed to LCD/Update it based on new mma data
    currentMillis = millis();
    speedChange(); 

    // --------+++= [Generate Servo Outputs] =+++--------
    yawServo.write(yawSpeed);
    pitchServo.write(pitchSpeed);
    if(flywheelEnabled) { flywheelServo.write(flywheelSpeed); }
    feedServo.write(feedPosition);
    frontWheel.write(leftWheelSpeed);
    backWheel.write(rightWheelSpeed);

    // Debugging output
    Serial.print("Yaw speed: "); Serial.print(yawSpeed);
    Serial.print(" | Pitch speed: "); Serial.print(pitchSpeed);
    Serial.print(" | X Tilt: "); Serial.print(xTilt);
    Serial.print(" | Left Wheel: "); Serial.print(leftWheelSpeed);
    Serial.print(" | Right Wheel: "); Serial.println(rightWheelSpeed);

    delay(10); // Small delay for stability
}

//==========================================HELPER FUNCTIONs==========================================

// Fires on 
void fire() {
  feedPosition = (feedPosition == 25) ? 130 : 25;
}

// Changes speed based on angle of controller
void speedChange() {
    // We check the mma values in the y axis to see if the controller is tilted forward or backwards
    // "Forwards" and "Backwards" are hardcoded angles in the first condition of the if conditions
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