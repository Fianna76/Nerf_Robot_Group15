#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

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
Servo rightWheel;

// Pin definitions
const int yawPin = 9;
const int pitchPin = 10;
const int feedPin = 6;
const int flywheelPin = 5;
const int frontWheelPin = 11;  // Left wheel servo
const int backWheelPin = 12; // Right wheel servo


// --------+++= [Joystick] =+++--------
const int joystickX = A0; // Yaw control
const int joystickY = A1; // Pitch control


// --------+++= [D-Pad] =+++--------
const int dpadUp = 4;
const int dpadDown = 7;
const int dpadLeft = 2;
const int dpadRight = 3;

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
    rightWheel.attach(backWheelPin);

    // Initialize servos to neutral position
    yawServo.write(90);
    pitchServo.write(90);
    feedServo.write(90);
    flywheelServo.write(0);
    frontWheel.write(90);
    rightWheel.write(90);

    Serial.println("Servos initialized..."); 

    pinMode(BCD_A, OUTPUT);
    pinMode(BCD_B, OUTPUT);
    pinMode(BCD_C, OUTPUT);
    pinMode(BCD_D, OUTPUT);

    speed = 0;


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
    
    // --------+++= [Update LCD] =+++--------
    //Display speed to LCD/Update it based on new mma data
    currentMillis = millis();
    speedChange();
      
    // Map joystick values (-100 to 100) using original logic
    if (joyX < 512) {
      yawMapped = map(joyX, 0, 512, 91, 180);
      yawSpeed = yawMapped;
    } else if (joyX > 512) {
      yawMapped = map(joyX, 512, 1023, 89, 0);
      yawSpeed = yawMapped;
    }
    if (joyY < 512) {
      pitchMapped = map(joyY, 0, 512, 91, 180);
      pitchSpeed = pitchMapped;
    } else if (joyY > 512) {
      pitchMapped = map(joyY, 512, 1023, 89, 0);
      pitchSpeed = pitchMapped;
    }

    

    // TODO: Remove trigger code

    // Read LT trigger for flywheel speed
    int ltValue = analogRead(ltTrigger);
    flywheelSpeed = constrain(map(ltValue, 25, 810, 165, 30), 30, 165);

    // Read RT trigger for feeding mechanism
    int rtValue = analogRead(rtTrigger);
    feedPosition = (rtValue < 130) ? 130 : 25;

    // Read accelerometer data
    mma.read();
    sensors_event_t event;
    mma.getEvent(&event);

    float xTilt = event.acceleration.x; // Left/Right tilt

    // Move wheels based on tilt
    if (xTilt > 4.0) {  // Tilted right
        leftWheelSpeed = 120;
        rightWheelSpeed = 60; // Opposite direction
    } else if (xTilt < -4.0) {  // Tilted left
        leftWheelSpeed = 60;
        rightWheelSpeed = 120; // Opposite direction
    } else {  // No tilt
        leftWheelSpeed = 88;
        rightWheelSpeed = 90;
    }

    // Write values to servos
    yawServo.write(yawSpeed);
    pitchServo.write(pitchSpeed);
    flywheelServo.write(flywheelSpeed);
    feedServo.write(feedPosition);
    frontWheel.write(leftWheelSpeed);
    rightWheel.write(rightWheelSpeed);

    // Debugging output
    Serial.print("Yaw speed: "); Serial.print(yawSpeed);
    Serial.print(" | Pitch speed: "); Serial.print(pitchSpeed);
    Serial.print(" | X Tilt: "); Serial.print(xTilt);
    Serial.print(" | Left Wheel: "); Serial.print(leftWheelSpeed);
    Serial.print(" | Right Wheel: "); Serial.println(rightWheelSpeed);

    delay(10); // Small delay for stability
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