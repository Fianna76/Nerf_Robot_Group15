#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <OneButton.h>

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

OneButton dpadUp(dpadUpPin, true); //False for Active HIGH pull-up
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
// Timing
unsigned long butPreviousMillis = 0;
unsigned long tiltPreviousMillis = 0;
#define BUT_INTERVAL (100UL) //Delay on movement of fire arm
#define TILT_INTERVAL (500UL)  // Delay speed changes (tickrate)

unsigned long currentMillis;

// Debounce
#define NUM_BUTTONS 1
#define DEBOUNCE_DELAY (15UL)
const int buttonPins[NUM_BUTTONS] = {3}; //IMPORTANT - ENSURE THESE ARE ALWAYS THE BUTTONS IN ORDER OF PIN DEFINITIONS
boolean buttonState[NUM_BUTTONS] = {true}; 
boolean lastButtonState[NUM_BUTTONS] = {true}; 
unsigned long debouncePreviousMillis[NUM_BUTTONS] = {0};

// Aiming
int yawSpeed = 88;      // Centered at 90 (neutral)
int pitchSpeed = 92;    // Centered at 90 (neutral)
int lastJoyX;
int lastJoyY;

boolean fineControl = false;

// Firing
int flywheelsSpeed = 0;
int feedPosition = 30;  // Default position
boolean flywheelsEnabled = true;
int potValue;

#define STARTPOS  30
#define FINALPOS 120
#define FIREDELAY (300UL)
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

// Debounces button inputs by checking against current system time
void debounce();

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
// Handles the release of button (turn off full auto)
void fireEnd();
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
    delay(500);
    flywheelsServo.write(150);
    delay(500);
    flywheelsServo.write(0);
    Serial.println("Armed!!");

    // Initialize Buttons
    // Set button pins as input with pull-up resistors
    // pinMode(dpadUpPin, INPUT_PULLUP);
    // pinMode(dpadDownPin, INPUT_PULLUP);
    // pinMode(dpadLeftPin, INPUT_PULLUP);
    // pinMode(dpadRightPin, INPUT_PULLUP); 


    // dpadUp.attachClick(fire);
    dpadUp.attachLongPressStart(fire);
    dpadUp.attachLongPressStop(fireEnd);
    dpadDown.attachClick(flywheelsToggle);
    dpadLeft.attachClick(joystickToggle); //TODO: Determine something for this/move fire integration
    dpadRight.attachClick(joystickToggle);

    

    Serial.println("Buttons initialized..."); 

    // Initialize BCD Encoder
    pinMode(BCD_A, OUTPUT);
    pinMode(BCD_B, OUTPUT);
    pinMode(BCD_C, OUTPUT);
    pinMode(BCD_D, OUTPUT);

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
    // TODO: Move repetive deadzone code to external function
    if (fineControl) {
      // Dynamic fine control range around last positions (max/min ensure we don't map outside our boundarys)
      int fineJoyX = map(joyX, 0, 1023, max(lastJoyX - 600, 0), min(lastJoyX + 600, 1023));
      int fineJoyY = map(joyY, 0, 1023, max(lastJoyY - 600, 0), min(lastJoyY + 600, 1023));
      if(fineJoyX > 520) {
        yawSpeed = map(joyX, 520, 1023, 92, 100);
      } else {
        yawSpeed = map(joyX, 0, 514, 80, 90);
      }
      if(fineJoyY > 515) {
        pitchSpeed = map(joyY, 515, 1023, 92, 100);
      } else {
        pitchSpeed = map(joyY, 0, 510, 77, 87);
      }
    } 
    else {
      // Full range mapping for large control
      if(joyX > 520) {
        yawSpeed = map(joyX, 520, 1023, 92, 180);
      } else {
        yawSpeed = map(joyX, 0, 514, 0, 90);
      }
      if(joyY > 515) {
        pitchSpeed = map(joyY, 515, 1023, 92, 180);
      } else {
        pitchSpeed = map(joyY, 0, 510, 0, 87);
      }
    } 

    // Read potentiometer for flywheels speed & map values
    if (flywheelsEnabled) { potValue = analogRead(potPin); } 
    else { potValue = 0; }
    flywheelsSpeed = map(potValue, 0, 1023, 30, 165);

    
    // --------+++= [Update LCD] =+++--------
    //Display speed to LCD/Update it based on new mma data
    currentMillis = millis();
    speedChange(); 
    directionChange();

    // --------+++= [Button Handling] =+++--------
    // Read buttons to see if there's an input - this could change values!
    debounce(); //Read buttons with debouncing of the inputs

    dpadUp.tick();
    fireLoop();
    dpadDown.tick();
    dpadLeft.tick();
    dpadRight.tick();

    // if (!buttonState[0]) { Serial.println("Up Touched"); fire(); }
    // if (!buttonState[1]) { Serial.println("Down Touched"); } //joystickToggle();
    // if (!buttonState[2]) { Serial.println("Left Touched");  stop(); }
    // if (!buttonState[3]) { Serial.println("Right Touched");  flywheelsToggle(); }

    // --------+++= [Generate Servo Outputs] =+++--------
    yawServo.write(yawSpeed);
    pitchServo.write(pitchSpeed);
    // feedServo.write(feedPosition);
    frontWheel.write(frontWheelSpeed);
    backWheel.write(backWheelSpeed);

    // Only write flywheels values out while its enabled (state controlled by button press)
    flywheelsServo.write(flywheelsSpeed);

    // Debugging output
    // Serial.println("");
    // Serial.print(" | Feed Pos | "); Serial.print(feedPosition);
    // Serial.print(" | JoyX | "); Serial.print(joyX);
    // Serial.print(" | JoyY | "); Serial.print(joyY);
    // Serial.print(" | Yaw speed: "); Serial.print(yawSpeed);
    // Serial.print(" | Pitch speed: "); Serial.print(pitchSpeed);
    // Serial.print(" | X Tilt: "); Serial.print(mma.x / 4096.0f);
    // Serial.print(" | Y Tilt: "); Serial.print(mma.y / 4096.0f);
    // Serial.print(" | Speed: "); Serial.print(speed);
    // Serial.print(" | Front Wheel: "); Serial.print(frontWheelSpeed);
    // Serial.print(" | Back Wheel: "); Serial.println(backWheelSpeed);
    // Serial.print("Pot Value: "); Serial.print(potValue);

    // Serial.println("JoystickX: "); Serial.print(joystickX);
    // Serial.print(" | JoystickY: "); Serial.print(joystickY);
}

//==========================================HELPER FUNCTIONs==========================================

void debounce()
{
  // Serial.println("");
  for (int i = 0; i < NUM_BUTTONS; i++) {
    bool reading = digitalRead(buttonPins[i]);  // Read button state
    // Serial.print(" | Button No "); Serial.print(i);
    

    if (reading != lastButtonState[i]) {
        debouncePreviousMillis[i] = millis();  // Reset debounce timer
        // Serial.print("NEW READING");
    }

    if ((millis() - debouncePreviousMillis[i]) > DEBOUNCE_DELAY) {
      // Edge detection
      if (reading != buttonState[i]) {
        buttonState[i] = reading;  // Update only if stable
    }
      
    }
    lastButtonState[i] = reading;  // Save last raw reading
    // Serial.print(" | State: | "); Serial.print(buttonState[i]);
    // Serial.print(" | Last state: "); Serial.print(reading);
  }
}


void stop()
{
  frontWheelSpeed = WHEEL_NEUTRAL;
  backWheelSpeed = WHEEL_NEUTRAL;
}

void fire() {
  
  Serial.println("Fire Touched");
  isFiring = true;

  if(currentMillis - butPreviousMillis >= BUT_INTERVAL)
  {
    
    butPreviousMillis = currentMillis;
  }
  
}

void fireEnd() {
  Serial.println("Fire released");
  isFiring = false;

} 

void fireLoop() {
  if (!isFiring) return; // Exit if the sequence isn't active

  switch (fireState) {
      case 0: // Move to STARTPOS
          feedPosition = STARTPOS;
          feedServo.write(feedPosition);
          firePreviousMillis = currentMillis;
          fireState = 1;
          Serial.println("State 0");
          break;

      case 1: // Wait, then move to FINALPOS
          if (currentMillis - firePreviousMillis >= FIREDELAY) {
            feedPosition = FINALPOS;  
            feedServo.write(feedPosition);
              firePreviousMillis = currentMillis;
              fireState = 2;
              Serial.println("State 1");
          }
          break;

      case 2: // Wait, then move back to STARTPOS
          if (currentMillis - firePreviousMillis >= FIREDELAY) {
            feedPosition = STARTPOS;  
            firePreviousMillis = currentMillis;
            feedServo.write(feedPosition);
                // isFiring = false; // Stop firing after cycle
                Serial.println("State 2");
                fireState = 0; // Reset state
                
          }
          break;
        }
}

// Toggles the opposite of the currrent enable condition 
void flywheelsToggle()
{
    Serial.println("flywheelsToggle touched | "); Serial.print("Enable Status: "); Serial.print(flywheelsEnabled);

    flywheelsEnabled = !flywheelsEnabled;
    flywheelsSpeed = 0;
    
}

void joystickToggle()
{
  Serial.println("JoystickToggle touched!");
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
    if (speed < 1 || speed > 4) { speed = 4; return;}
    
    // Detect forward rotation (Increase Speed)
    if((mma.y / 4096.0f) <= -0.75f && currentMillis - tiltPreviousMillis >= TILT_INTERVAL && speed < 4)
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
    backWheelSpeed = 89;
  }
}