/* 
   A closed-loop proportional control system
   gkn 20230209
*/

// Include the necessary library for Pin Change Interrupts
// This library is available via the Arduino Library Manager
#include <PinChangeInterrupt.h>

// Motor driver definitions

// SHIELD variable: Set to true if using the moto shield, false if using a Dual H-Bridge controller without the shield
#define SHIELD false

// Shield pin definitions (cannot be changed)
#define motorApwm 3  // PWM pin for motor A
#define motorAdir 12 // Direction pin for motor A
#define motorBpwm 11 // PWM pin for motor B
#define motorBdir 13 // Direction pin for motor B

// Driver pin definitions for Dual H-Bridge (analog pins marked with ~ on your board)
#define IN1 9  // IN1 pin for motor driver
#define IN2 10 // IN2 pin for motor driver
#define IN3 5  // IN3 pin for motor driver
#define IN4 6  // IN4 pin for motor driver

// Lab specific definitions

// Constants to represent motor A and B for easier reference in the code
#define A 0
#define B 1

// Pin for a push button connected to pin 11 (with internal pull-up resistor)
#define pushButton 11 

// Pins for wheel encoders (left and right motors)
#define EncoderMotorLeft  7  // Pin for the left encoder
#define EncoderMotorRight 8  // Pin for the right encoder

// Encoder counts for left and right motors, declared as volatile since they are updated in interrupt service routines
volatile unsigned int leftEncoderCount = 0;
volatile unsigned int rightEncoderCount = 0;

// Move definitions for different directions
#define FORWARD             0
#define LEFT                1
#define RIGHT              -1

// Constants for robot drive configuration
#define EncoderCountsPerRev 24.0 // Encoder counts per wheel revolution
#define DistancePerRev      28.0 // Distance in CM per wheel revolution
#define DegreesPerRev       22.3 // Degrees turned by the robot per wheel revolution

// Proportional control constants
#define GAIN_A 5          // Gain for motor A (right motor)
#define GAIN_B 4.2        // Gain for motor B (left motor)
#define turnGainA 1.7     // Gain for turning (motor A)
#define turnGainB 1       // Gain for turning (motor B)
#define distTolerance 1   // Distance tolerance (acceptable error for encoder counts)

// Deadband power settings (minimum PWM to keep the wheels moving)
#define deadband_A 75 // Minimum PWM for motor A
#define deadband_B 50 // Minimum PWM for motor B

// Define the movement sequence (forward, left, right, etc.) for navigating a maze
int moves[] = {FORWARD, LEFT, FORWARD, LEFT, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD}; 
// Define the distance to travel for each move
int distance[] = {30, 30, 30, 100, 70, 30};

void setup() {
  // Initialize the serial communication and motor setup
  Serial.begin(9600);
  motor_setup(); // Function to set up the motor pins
  pinMode(pushButton, INPUT_PULLUP); // Configure the push button with a pull-up resistor

  // Print message and set up encoders for both motors
  Serial.print("Encoder Testing Program ");
  Serial.print("Now setting up the Left Encoder: Pin ");
  Serial.print(EncoderMotorLeft);
  Serial.println();
  pinMode(EncoderMotorLeft, INPUT_PULLUP); // Set left encoder pin as input
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(EncoderMotorLeft), indexLeftEncoderCount, CHANGE); // Set interrupt for left encoder

  Serial.print("Now setting up the Right Encoder: Pin ");
  Serial.print(EncoderMotorRight);
  Serial.println();
  pinMode(EncoderMotorRight, INPUT_PULLUP); // Set right encoder pin as input
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(EncoderMotorRight), indexRightEncoderCount, CHANGE); // Set interrupt for right encoder
}

void loop() {
  int j = 0;

  // Wait for the push button to be pressed and released
  while (digitalRead(pushButton) == 1); // Wait for button press
  delay(50); // Debounce delay
  while (digitalRead(pushButton) == 0); // Wait for button release

  // Loop through the move sequence and execute the corresponding actions
  for (int i = 0; i < sizeof(moves)/2; i++) { 
    if(moves[i] == FORWARD){
      drive(distance[j]); // Drive forward for the specified distance
      j++;
    }
    else {
      Turn(moves[i]); // Turn in the specified direction (left or right)
    }
    // Stop the motors after each move
    run_motor(A, 0);
    run_motor(B, 0);
  }
  j = 0; // Reset the distance index
}

// Function to drive the robot forward for a specified distance
int drive(float distance) {
  int countsDesired, cmdLeft, cmdRight, errorLeft, errorRight;

  // Calculate the number of encoder counts required to cover the given distance
  countsDesired = (distance / DistancePerRev) * EncoderCountsPerRev;

  // Reset the current encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  // Initialize the errors to values greater than tolerance to enter the loop
  errorLeft = distTolerance + 1;
  errorRight = distTolerance + 1;

  // Proportional control loop
  while (errorLeft > distTolerance && errorRight > distTolerance) {
    if (errorLeft > distTolerance) {
      cmdLeft = proportionalControl(GAIN_A, deadband_A, errorLeft, LEFT); // Control motor A
    }
    if (errorRight > distTolerance) {
      cmdRight = proportionalControl(GAIN_B, deadband_B, errorRight, RIGHT); // Control motor B
    }

    // Set the motor PWMs
    run_motor(A, cmdLeft);
    run_motor(B, cmdRight);

    // Update the encoder error
    errorLeft = countsDesired - leftEncoderCount; 
    errorRight = countsDesired - rightEncoderCount;
  }

  // Stop the motors
  run_motor(A, 0);
  run_motor(B, 0);
}

// Proportional control function for motor speed adjustment
int proportionalControl(int gain, int deadband, int error, int motor) {
  int max;
  if (error <= distTolerance) { // If error is within tolerance, stop motor
    return 0;
  }
  if (motor == LEFT) {
    max = 170; // Maximum PWM for motor A
  }
  else {
    max = 120; // Maximum PWM for motor B
  }
  int pwm = gain * error; // Calculate proportional control
  pwm = constrain(pwm, deadband, max); // Constrain PWM between deadband and max
  return pwm;
}

// Function to turn the robot by a specified angle (using encoder counts)
unsigned long Turn(int sign) {
  int leftCount = 11; // Placeholder encoder counts for turns
  int rightCount = 11;
  int errorLeft = 2;
  int errorRight = 2;
  int Gainleft, Gainright;

  // Reset current encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  // If turning left
  if (sign == LEFT) {
    while (errorLeft > distTolerance && errorRight > distTolerance) {
      if (errorLeft > distTolerance) {
        errorLeft = leftCount - leftEncoderCount;
        Gainleft = proportionalControl(turnGainA, deadband_A, errorLeft, LEFT); // Control motor A for left turn
      }
      if (errorRight > distTolerance) {
        errorRight = rightCount - rightEncoderCount;
        Gainright = proportionalControl(turnGainB, deadband_B, errorRight, RIGHT); // Control motor B for left turn
      }

      // Apply the calculated PWM values for turning
      run_motor(A, -Gainleft); // Reverse motor A for left turn
      run_motor(B, Gainright); // Forward motor B for left turn
    }
    run_motor(A, 0); // Stop motors after turn
    run_motor(B, 0);
  }
  // If turning right
  else if (sign == RIGHT) {
    while (errorRight > distTolerance && errorLeft > distTolerance) {
      if (errorRight > distTolerance) {
        errorRight = rightCount - rightEncoderCount;
        Gainright = proportionalControl(turnGainB, deadband_B, errorRight, RIGHT); // Control motor B for right turn
      }
      if (errorLeft > distTolerance) {
        errorLeft = leftCount - leftEncoderCount;
        Gainleft = proportionalControl(turnGainA, deadband_A, errorLeft, LEFT); // Control motor A for right turn
      }

      // Apply the calculated PWM values for turning
      run_motor(A, Gainleft); // Forward motor A for right turn
      run_motor(B, -Gainright); // Reverse motor B for right turn
    }
    run_motor(A, 0); // Stop motors after turn
    run_motor(B, 0);
  }
}

// Interrupt service routine for left encoder count (triggered on pin change)
void indexLeftEncoderCount() {
  leftEncoderCount++; // Increment left encoder count
}

// Interrupt service routine for right encoder count (triggered on pin change)
void indexRightEncoderCount() {
  rightEncoderCount++; // Increment right encoder count
}
