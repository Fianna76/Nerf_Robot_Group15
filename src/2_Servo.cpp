#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>


// Joystick pins
int joystickXPin = A0;
int joystickYPin = A1;

// Deadzone settings
int deadzone1 = 5, deadzone2 = -5, stopVal = 90;
int minSpeed = -60, maxSpeed = 60;
int rotationServoPos, tiltServoPos;

// Joystick values
int joystickXVal, joystickYVal;

// Servo pins
int rotationServoPin = 5;
int tiltServoPin = 3;
int firingServoPin = 10;
int ButtonPin = 4;

// Servo objects
Servo rotationServo, tiltServo, firingServo;

// Firing mechanism
int startPos = 90, angleRange = 10, delayTime = 5;

// Function to setup the joystick
void joystickSetup() {
  pinMode(joystickXPin, INPUT);
  pinMode(joystickYPin, INPUT);
  rotationServo.attach(rotationServoPin);
  tiltServo.attach(tiltServoPin);
}

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
  Serial.print("Rotation: "); Serial.print(rotationServoPos);
  Serial.print(" | Tilt: "); Serial.println(tiltServoPos);
}

// Function to handle firing mechanism
void firingLoop(bool buttonPressed) {
  if (buttonPressed) {
    for (int pos = 80; pos <= 100; pos++) {
      firingServo.write(pos);
      delay(delayTime);
    }
    for (int pos = 100; pos >= 80; pos--) {
      firingServo.write(pos);
      delay(delayTime);
    }
  }
  firingServo.write(90); 
}

void setup() {
  // Setup joystick and servos
  joystickSetup();
  pinMode(ButtonPin, INPUT_PULLUP);
  firingServo.attach(firingServoPin);
  firingServo.write(90);

  // Serial connection for debugging
  Serial.begin(9600);
}

void loop() {
  // Read joystick values
  joystickXVal = analogRead(joystickXPin);
  joystickYVal = analogRead(joystickYPin);

  // Control firing mechanism
  firingLoop(digitalRead(ButtonPin));

  // Control turret movement
  joystickLoop(joystickXVal, joystickYVal);
}