#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
// #include <OneButton.h>

// ========+++- [Global Variable Setup] -+++========

// --------+++= [MMA8451] =+++--------
// Initialize MMA8451 accelerometer at address 0x1D
Adafruit_MMA8451 mma = Adafruit_MMA8451(0x1C);

// --------+++= [Servos] =+++--------

// Servo Objects
Servo yawServo;
Servo pitchServo;
Servo feedServo;
Servo flywheelsServo;
Servo frontWheel;
Servo backWheel;

// Pin definitions
const int yawPin = 9;
const int pitchPin = 10;
const int feedPin = 6;
const int flywheelsPin = 5;
const int frontWheelPin = 11;  
const int backWheelPin = 12; 

// --------+++= [Analogue Inputs] =+++--------
const int joystickX = A0; // Yaw control
const int joystickY = A1; // Pitch control

const int potPin = A2; //Potentiometer 

// --------+++= [D-Pad] =+++--------
const int dpadUpPin = 3;
const int dpadDownPin = 4;
const int dpadLeftPin = 2;
const int dpadRightPin = 7;

// OneButton dpadUp(dpadUpPin, false); //False for Active HIGH pull-up
// OneButton dpadDown(dpadDownPin, false);
// OneButton dpadLeft(dpadLeftPin, false);
// OneButton dpadRight(dpadRightPin, false);

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
// Timing
unsigned long butPreviousMillis = 0;
unsigned long tiltPreviousMillis = 0;
#define BUT_INTERVAL (250UL)
#define TILT_INTERVAL (500UL)  // Delay speed changes (tickrate)

unsigned long currentMillis;

// Aiming
int yawSpeed = 90;      // Centered at 90 (neutral)
int pitchSpeed = 90;    // Centered at 90 (neutral)
int lastJoyX;
int lastJoyY;

boolean fineControl = false;

// Firing
int flywheelsSpeed = 0;
int feedPosition = 25;  // Default position
boolean flywheelsEnabled = true;

#define STARTPOS  25
#define FINALPOS 130 
#define FIREDELAY (150UL)
int fireState = 0;
bool isFiring = false;
unsigned long firePreviousMillis = 0;

// Movment
#define WHEEL_TOP_SPEED 120
#define WHEEL_BOTTOM_SPEED 60
#define WHEEL_NEUTRAL 90

int frontWheelSpeed = 90;
int backWheelSpeed = 90;
int speed = 1; // 7Seg Display

// ========+++- [Helper Function Declarations] -+++========

// Changes speed based on angle of controller
void speedChange();

//Updates the direction to move based on the angle of the controller
void directionChange();

// Stops all motion on button press
void stop();

// Toggles flywheels input from potentiometer (on button press)
void flywheelsToggle();

// Toggles between Micro/Macro adjustments of aim with joystick (on button press)
void joystickToggle();

// Fires a single bullet on button press
void fire();
// Handles the position changes of the firing arm
void fireLoop();

//==============================================SETUP==============================================
void setup() {
    Serial.begin(9600); //TODO: At 4600 only for testing
    Serial.println("Initializing system...");

    // Attach servos
    yawServo.attach(yawPin);
    pitchServo.attach(pitchPin);
    feedServo.attach(feedPin);
    flywheelsServo.attach(flywheelsPin);
    frontWheel.attach(frontWheelPin);
    backWheel.attach(backWheelPin);

    // Initialize servos to neutral position
    yawServo.write(90);
    pitchServo.write(90);
    feedServo.write(90);
    flywheelsServo.write(0);
    frontWheel.write(90);
    backWheel.write(90);

    Serial.println("Servos initialized..."); 

    Serial.println("Arming Sequence Begin...");
    flywheelsServo.write(45);
    delay(1000);
    flywheelsServo.write(0);
    Serial.println("Armed!!");

    // Initialize Buttons
    // Set button pins as input with pull-up resistors
    pinMode(dpadUpPin, INPUT_PULLUP);
    pinMode(dpadDownPin, INPUT_PULLUP);
    pinMode(dpadLeftPin, INPUT_PULLUP);
    pinMode(dpadRightPin, INPUT_PULLUP); 


    // TODO: Test if onebutton actually works!
    // dpadUp.attachClick(fire); //Single Click 


    // dpadDown.attachClick(flywheelsToggle);

   
    // // dpadLeft.attachClick(fire); //TODO: Determine something for this/move fire integration

  
    // dpadRight.attachClick(joystickToggle);

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
    // // Read new tilt data
    mma.read();

    currentMillis = millis();

    // Read joystick values
    int joyX = analogRead(joystickX);
    int joyY = analogRead(joystickY);
    
    // Map joystick values
    if (fineControl) {
      // Dynamic fine control range around last positions (max/min ensure we don't map outside our boundarys)
      int fineJoyX = map(joyX, 0, 1023, max(lastJoyX - 200, 0), min(lastJoyX + 200, 1023));
      int fineJoyY = map(joyY, 0, 1023, max(lastJoyY - 200, 0), min(lastJoyY + 200, 1023));

      yawSpeed = map(fineJoyX, 0, 1023, 0, 180);
      pitchSpeed = map(fineJoyY, 0, 1023, 0, 180);
    } 
    else {
      // Full range mapping for large control
      yawSpeed = map(joyX, 0, 1023, 0, 180);
      pitchSpeed = map(joyY, 0, 1023, 0, 180);
    } 

    // Read potentiometer for flywheels speed & map values
    int potValue = analogRead(potPin);
    flywheelsSpeed = map(potValue, 0, 1023, 30, 165);

    
    // --------+++= [Update LCD] =+++--------
    //Display speed to LCD/Update it based on new mma data
    currentMillis = millis();
    speedChange(); 
    directionChange();

    // --------+++= [Button Handling] =+++--------
    // Read buttons to see if there's an input - this could change values!
    if (digitalRead(dpadUpPin) == LOW) { Serial.println("Up Touched"); fire(); }
     if (digitalRead(dpadDownPin) == LOW) { Serial.println("Down Touched"); joystickToggle(); }
    if (digitalRead(dpadLeftPin) == LOW) { Serial.println("Left Touched");  stop(); }
    if (digitalRead(dpadRightPin) == LOW) { Serial.println("Right Touched");  flywheelsToggle(); }

    // --------+++= [Generate Servo Outputs] =+++--------
    yawServo.write(yawSpeed);
    pitchServo.write(pitchSpeed);
    // feedServo.write(feedPosition);
    frontWheel.write(frontWheelSpeed);
    backWheel.write(backWheelSpeed);

    // Only write flywheels values out while its enabled (state controlled by button press)
    if(flywheelsEnabled) { flywheelsServo.write(flywheelsSpeed);}

    // Debugging output
    // Serial.println("");
    // Serial.print("Yaw speed: "); Serial.print(yawSpeed);
    // Serial.print(" | Pitch speed: "); Serial.print(pitchSpeed);
    // Serial.print(" | X Tilt: "); Serial.print(mma.x / 4096.0f);
    // Serial.print(" | Y Tilt: "); Serial.print(mma.y / 4096.0f);
    // Serial.print(" | Speed: "); Serial.print(speed);
    // Serial.print(" | Front Wheel: "); Serial.print(frontWheelSpeed);
    // Serial.print(" | Back Wheel: "); Serial.println(backWheelSpeed);
    // Serial.print("Pot Value: "); Serial.print(potValue);

    // Serial.println("JoystickX: "); Serial.print(joystickX);
    // Serial.print(" | JoystickY: "); Serial.print(joystickY);

    delay(10); // Small delay for stability
}

//==========================================HELPER FUNCTIONs==========================================

// Fires on button press - might not work with click, try integrating press/release
void stop()
{
  
  frontWheelSpeed = WHEEL_NEUTRAL;
  backWheelSpeed = WHEEL_NEUTRAL;
}

void fire() {
  Serial.println("Fire Touched");
  
  if(currentMillis - butPreviousMillis >= BUT_INTERVAL)
  {

    isFiring = true;
    fireLoop();
    butPreviousMillis = currentMillis;
  }
  
}

void fireLoop() {
  if (!isFiring) return; // Exit if the sequence isn't active

  switch (fireState) {
      case 0: // Move to STARTPOS
          feedServo.write(STARTPOS);
          firePreviousMillis = currentMillis;
          fireState = 1;
          break;

      case 1: // Wait, then move to FINALPOS
          if (currentMillis - firePreviousMillis >= FIREDELAY) {
              feedServo.write(FINALPOS);
              firePreviousMillis = currentMillis;
              fireState = 2;
          }
          break;

      case 2: // Wait, then move back to STARTPOS
          if (currentMillis - firePreviousMillis >= FIREDELAY) {
              feedServo.write(STARTPOS);
                isFiring = false; // Stop firing after cycle
                fireState = 0; // Reset state
          }
          break;
        }
}

// Toggles the opposite of the currrent enable condition 
void flywheelsToggle()
{
  if(currentMillis - butPreviousMillis >= BUT_INTERVAL)
  {
    Serial.println("Right Clicked!"); Serial.print("Enable Status: "); Serial.print(flywheelsEnabled);
    flywheelsEnabled = !flywheelsEnabled;
    flywheelsSpeed = 0;
    butPreviousMillis = currentMillis;
  }
  
}

void joystickToggle()
{
  fineControl = !fineControl;

  if(fineControl)
  {
    lastJoyX = analogRead(joystickX);
    lastJoyY = analogRead(joystickY);
  }
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
    
    // Ensure digit is within valid range
    if (speed < 1 || speed > 4) { speed = 1; return;}
    
    // Detect forward rotation (Increase Speed)
    if((mma.y / 4096.0f) <= -0.75f && currentMillis - tiltPreviousMillis >= TILT_INTERVAL && speed < 9)
    {
        speed++;
        tiltPreviousMillis = currentMillis;
        // Serial.print("Speed up: "); Serial.println(speed);
    }
    // Backwards rotation decreases speed
    else if((mma.y / 4096.0f) >= 0.75f && currentMillis - tiltPreviousMillis >= TILT_INTERVAL && speed > 0)
    {
        speed--;
        tiltPreviousMillis = currentMillis;
        // Serial.print("Speed down: "); Serial.println(speed);
    }

    

    // Write the BCD bits to the 74LS47 inputs
    digitalWrite(BCD_A, bcdLookup[speed][3]);
    digitalWrite(BCD_B, bcdLookup[speed][2]);
    digitalWrite(BCD_C, bcdLookup[speed][1]);
    digitalWrite(BCD_D, bcdLookup[speed][0]);

    // Serial.println(speed);
}

// Set the speed of our output to our wheels based off the tilt sensor direction and value of "speed"
void directionChange()
{
  // Map our speed to the actual PWM values we send out
  int maxSpeed = map(speed,1,4,WHEEL_NEUTRAL,WHEEL_TOP_SPEED);
  int minSpeed = map(speed,1,4,WHEEL_NEUTRAL,WHEEL_BOTTOM_SPEED);
  
  // Right
  if((mma.x / 4096.0f) >= 0.5f) {
      frontWheelSpeed = maxSpeed;
      backWheelSpeed = minSpeed;
  }
  // Left
  else if((mma.x / 4096.0f) <= -0.5)
  {
      frontWheelSpeed = minSpeed;
      backWheelSpeed = maxSpeed;
  }
  // Neutral
  else
  {
    frontWheelSpeed = WHEEL_NEUTRAL;
    backWheelSpeed = WHEEL_NEUTRAL;
  }
}