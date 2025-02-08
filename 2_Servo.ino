#include <Servo.h>
//All joystick values
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
//joystick values
int joystickXVal;
int joystickYVal;
int joystickButtonState;
//servos associated with joystick
int rotationServoPin= 5;
int tiltServoPin = 3;
int firingServoPin = 10;
int rotationServoPos;
int tiltServoPos;
// maximum absolute value is 90
int maxSpeed = 45, minSpeed = -45;
//servos for rotation and tilt of turret
Servo rotationServo, tiltServo, firingServo;
//deadzones for joystick drift
int deadzone1 = 5, deadzone2 = -5, stopVal = 0;




//function to setup the joystick
void joystickSetup() {
  //pin for joysticks and attaching servos to pins
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  rotationServo.attach(xServo);
  tiltServo.attach(yServo);
}

void joystickLoop(int xVal, int yVal, int buttonState) {
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

void setup() {
  //setup
  joystickSetup();
  //serial connection for debugging
  Serial.begin(9600);
}

void loop() {
  //read the pins of the joystick
  joystickXVal = analogRead(xPin);
  joystickYVal = analogRead(yPin);
  joystickButtonState = digitalRead(buttonPin);
  joystickLoop(joystickXVal, joystickYVal, joystickButtonState);

  //TO BE CHANGED
  //using joystick button for firing servo, assuming the spin mechanism is 360 degrees.
  if (joystickButtonState == LOW) {
    firingServo.write(90);
  } else {
    firingServo.write(0);
  }
}