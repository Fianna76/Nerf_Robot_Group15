#include <Servo.h>

//joystick pins
int xPin = A0;
int yPin = A1;
int buttonPin = 2;


//joystick values
int xVal;
int yVal;
int buttonState;


//servos associated with joystick
int xServo = 5;
int yServo = 3;
int xServoPos;
int yServoPos;

// maximum absolute value is 90
int maxSpeed = 45, minSpeed = -45;

//servos for rotation and tilt of turret
Servo rotationServo, tiltServo;

//deadzones for joystick drift
int deadzone1 = 5, deadzone2 = -5, stopVal = 0;

void setup() {
  //pin for joysticks and attaching servos to pins
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  rotationServo.attach(xServo);
  tiltServo.attach(yServo);

  //serial connection for debugging
  Serial.begin(4800);
}

void loop() {

  xVal = analogRead(xPin);
  yVal = analogRead(yPin);
  buttonState = digitalRead(buttonPin);

  //0 is still, 90 max speed clockwise, 180 max speed counter-clockwise
  xServoPos = map(xVal, 0, 1023, minSpeed, maxSpeed);
  yServoPos = map(yVal, 0, 1023, minSpeed, maxSpeed);

  //make sure the servos dont move when the joystick is in the center position
  if (xServoPos > deadzone2 && xServoPos < deadzone1) {
    xServoPos = stopVal;
  }
  if (yServoPos > deadzone2 && yServoPos < deadzone1) {
    yServoPos = stopVal;
  }

  //the servos switch directions on > 90 rather than negative so we add 180 to negative values to get other direction.
  if (xServoPos < deadzone2) {
    rotationServo.write(xServoPos + 180);
  } else {
    rotationServo.write(xServoPos);
  }
  if (yServoPos < deadzone2) {
    tiltServo.write(yServoPos + 180);
  } else {
    tiltServo.write(yServoPos);
  }

  // Debugging output
  Serial.print("Rotation: "); Serial.print(xServoPos);
  Serial.print(" | Tilt: "); Serial.println(yServoPos);
}