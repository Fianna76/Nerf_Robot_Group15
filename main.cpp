#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

//TODO: I'm factoring this code to work off my UNO cause that's how my IDE is configured right now
//      Need to change the pinouts to work with the NANO 

// Create an MMA8451 instance - tilt sensor 
Adafruit_MMA8451 mma = Adafruit_MMA8451();

// mma pins
int SCL = A5; // Serial Clock
int SDA = A4; // Serial Data

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
  // Serial connection for debugging
  Serial.begin(9600);
  
  // Setup joystick and servos
  joystickSetup();
  pinMode(ButtonPin, INPUT_PULLUP);
  firingServo.attach(firingServoPin);
  firingServo.write(90);

   // Initialize MMA8451
   if (!mma.begin()) 
   {
      Serial.println("Could not find MMA8451 accelerometer. Check wiring!");
      while (1);
  }

  Serial.println("MMA8451 found!");

  // Set sensor to 2G range (can be 2, 4, or 8G) | TODO: Decide what range we want - 2G should be fine though
  mma.setRange(MMA8451_RANGE_2_G); 
  
}

void loop() {
  // Read joystick values
  joystickXVal = analogRead(joystickXPin);
  joystickYVal = analogRead(joystickYPin);

  // Control firing mechanism
  firingLoop(digitalRead(ButtonPin));

  // Control turret movement
  joystickLoop(joystickXVal, joystickYVal);

  // Tilt / track movement
  tiltLoop();
}

void tiltLoop()
{
    tiltRead();
}

void tiltRead()
{
    // Read new data
    mma.read();
    
    // Get acceleration values (X, Y, Z)
    Serial.print("X: "); Serial.print(mma.x / 4096.0); Serial.print(" m/s²");
    Serial.print(" | Y: "); Serial.print(mma.y / 4096.0); Serial.print(" m/s²");
    Serial.print(" | Z: "); Serial.print(mma.z / 4096.0); Serial.println(" m/s²");

    //TODO: Determine ideal delay - 9600 Baud is probably a much higher refresh rate then we actually need for this
    delay(500);
}