#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// ========+++- [Global Variable Setup] -+++========

// --------+++= [MMA8451] =+++--------
// Initialize MMA8451 accelerometer at address 0x1D
Adafruit_MMA8451 mma = Adafruit_MMA8451(0x1D);

// --------+++= [Servos] =+++--------

// Servo Objects
Servo yawServo;
Servo pitchServo;
Servo feedServo;
Servo flywheelESC;
Servo leftWheel;
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

// --------+++= [Global Variables] =+++--------
int yawSpeed = 90;      // Centered at 90 (neutral)
int pitchSpeed = 90;    // Centered at 90 (neutral)
int flywheelSpeed = 0;
int feedPosition = 25;  // Default position
int leftWheelSpeed = 90;
int rightWheelSpeed = 90;
int yawMapped = 0;
int pitchMapped = 0;

//==============================================SETUP==============================================
void setup() {
    Serial.begin(4800); //TODO: At 4600 only for testing
    Serial.println("Initializing system...");

    // Attach servos
    yawServo.attach(yawPin);
    pitchServo.attach(pitchPin);
    feedServo.attach(feedPin);
    flywheelESC.attach(flywheelPin);
    leftWheel.attach(frontWheelPin);
    rightWheel.attach(backWheelPin);

    // Initialize servos to neutral position
    yawServo.write(90);
    pitchServo.write(90);
    feedServo.write(90);
    flywheelESC.write(0);
    leftWheel.write(90);
    rightWheel.write(90);

    Serial.println("Servos initialized..."); 

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
    // Read joystick values
    // int joyX = analogRead(joystickX);
    // int joyY = analogRead(joystickY);  
    //REDUNDANT CODE TO REMOVE - JOYSTICK UNUSED NOW
    // Map joystick values (-100 to 100) using original logic
    // if (joyX < 512) {
    //   yawMapped = map(joyX, 0, 512, 91, 180);
    //   yawSpeed = yawMapped;
    // } else if (joyX > 512) {
    //   yawMapped = map(joyX, 512, 1023, 89, 0);
    //   yawSpeed = yawMapped;
    // }
    // if (joyY < 512) {
    //   pitchMapped = map(joyY, 0, 512, 91, 180);
    //   pitchSpeed = pitchMapped;
    // } else if (joyY > 512) {
    //   pitchMapped = map(joyY, 512, 1023, 89, 0);
    //   pitchSpeed = pitchMapped;
    // }

    // Read D-pad buttons for fine adjustments
    if (digitalRead(dpadUp) == LOW) pitchSpeed += 5;
    if (digitalRead(dpadDown) == LOW) pitchSpeed -= 5;
    if (digitalRead(dpadLeft) == LOW) yawSpeed -= 5;
    if (digitalRead(dpadRight) == LOW) yawSpeed += 5;

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
    flywheelESC.write(flywheelSpeed);
    feedServo.write(feedPosition);
    leftWheel.write(leftWheelSpeed);
    rightWheel.write(rightWheelSpeed);

    // Debugging output
    Serial.print("Yaw speed: "); Serial.print(yawSpeed);
    Serial.print(" | Pitch speed: "); Serial.print(pitchSpeed);
    Serial.print(" | ltTrigger: "); Serial.print(ltTrigger);
    Serial.print(" | rtTrigger: "); Serial.print(rtTrigger);
    Serial.print(" | X Tilt: "); Serial.print(xTilt);
    Serial.print(" | Left Wheel: "); Serial.print(leftWheelSpeed);
    Serial.print(" | Right Wheel: "); Serial.println(rightWheelSpeed);

    delay(10); // Small delay for stability
}