/* a closed loop proportional control
   gkn 20230209
*/

// Include Libraries
// This is the PinChangeInterrupt package by NicoHood, 
//      available in the library search (Sketch->Include Libraries->Manage Libraries)
#include <PinChangeInterrupt.h>

// Motor driver definitions

// If you have a kit with the moto shield, set this to true
// If you have the Dual H-Bridge controller w/o the shield, set to false
#define SHIELD true
//SHIELD Pin varables - cannot be changed
#define motorApwm 3
#define motorAdir 12
#define motorBpwm 11
#define motorBdir 13
//Driver Pin variable - any 4 analog pins (marked with ~ on your board)
#define IN1 9
#define IN2 10
#define IN3 5
#define IN4 6

// Lab Specific definitions

// Defining these allows us to use letters in place of binary when
// controlling our motors
#define A 0
#define B 1
#define pushButton 2 // install a Pullup button with its output into Pin 2

// You may want to define pins for buttons as bump sensors. Pay attention to other used pins.


// The digital pins we'll use for the encoders
#define EncoderMotorLeft  7
#define EncoderMotorRight 8

// Initialize encoder counts to 0
// These are volatile because they change during interrupt functions
volatile unsigned int leftEncoderCount = 0;
volatile unsigned int rightEncoderCount = 0;

//These are to build your moves array, a la Lab 2
#define FORWARD             0
#define LEFT                1
#define RIGHT              -1

// CONSTANTS TO MODIFY IN THIS LAB

// Drive constants - dependent on robot configuration
#define EncoderCountsPerRev 24.0 // Encoder counts per wheel revolution
#define DistancePerRev      22.3 // Distance in CM per wheel revolution
#define DegreesPerRev       22.3 // Degrees turned by your robot per wheel revolution

// Proportional Control constants
// what are your ratios of PWM:Encoder Count error?
#define GAIN_A 5
#define GAIN_B 5
// how many encoder counts from your goal are accepteable?
#define distTolerance 3 

// Deadband power settings
// The min PWM required for your robot's wheels to still move
// May be different for each motor
#define deadband_A 20
#define deadband_B 20

// Lab specific variables
int moves[] = {}; // Fill in this array will forward distances and turn directions in the maze (Like part A)

void setup() {
  // set stuff up
  Serial.begin(9600);
  motor_setup();
  pinMode(pushButton, INPUT_PULLUP);
  
  // add additional pinMode statements for any bump sensors
  

  // Attaching Wheel Encoder Interrupts
  Serial.print("Encoder Testing Program ");
  Serial.print("Now setting up the Left Encoder: Pin ");
  Serial.print(EncoderMotorLeft);
  Serial.println();
  pinMode(EncoderMotorLeft, INPUT_PULLUP); //set the pin to input

  // The following code sets up the PinChange Interrupt
  // Valid interrupt modes are: RISING, FALLING or CHANGE
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(EncoderMotorLeft), indexLeftEncoderCount, CHANGE);
  // if you "really" want to know what's going on read the PinChange.h file :)
  /////////////////////////////////////////////////
  Serial.print("Now setting up the Right Encoder: Pin ");
  Serial.print(EncoderMotorRight);
  Serial.println();
  pinMode(EncoderMotorRight, INPUT_PULLUP);     //set the pin to input
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(EncoderMotorRight), indexRightEncoderCount, CHANGE);
}



/////////////////////// loop() ////////////////////////////////////
void loop()
{

  while (digitalRead(pushButton) == 1); // wait for button push
  delay(50); // Allow button time to debounce.
  while (digitalRead(pushButton) == 0); // wait for button release

  for (int i = 0; i < sizeof(moves)/2; i++) { // Loop through entire moves list
    if(moves[i]==LEFT){
      // Fill with code to turn left
    }
    else if(moves[i]==RIGHT){
      // Fill with code to turn right
    }
    else{
      // Fill with code to drive forward
    }
    run_motor(A, 0);
    run_motor(B, 0);
    delay(1000);
  }
}
//////////////////////////////// end of loop() /////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
int drive(float distance)
{
  // create variables needed for this function
  int countsDesired, cmdLeft, cmdRight, errorLeft, errorRight;

  // TODO: Find the number of encoder counts based on the distance given, and the 
  // configuration of your encoders and wheels
  countsDesired = ;

  // reset current encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  
  // we temporarily set the errors greater than our tolerance so our first test gets us into the loop
  errorLeft = distTolerance + 1;
  errorRight =  distTolerance + 1;

  // Begin proportional control until move is complete
  while (errorLeft > distTolerance || errorRight > distTolerance)
  {
    // Get PWM values from proportionalControl function
    cmdLeft =  proportionalControl(GAIN_A, deadband_A, errorLeft);
    cmdRight = proportionalControl(GAIN_B, deadband_B, errorRight);

    // Set new PWMs
    run_motor(A, cmdLeft);
    run_motor(B, cmdRight);

    // Update encoder error
    // Error is the number of encoder counts between here and the destination
    errorLeft = ; // TODO
    errorRight = ; //TODO
    
    /* Some print statements, for debugging
    Serial.print(errorLeft);
    Serial.print(" ");
    Serial.print(cmdLeft);
    Serial.print("\t");
    Serial.print(errorRight);
    Serial.print(" ");
    Serial.println(cmdRight);
    */
  }

}
////////////////////////////////////////////////////////////////////////////////


// Write a function for turning with PID control, similar to the drive function


//////////////////////////////////////////////////////////

int proportionalControl(int gain, int deadband, int error)
//  gain, deadband, and error, both are integer values
{
  if (error <= distTolerance) { // if error is acceptable, PWM = 0
    return (0);
  }

  int pwm = (gain * error); // Proportional control
  pwm = constrain(cmdDir,deadband,255); // Bind value between motor's min and max
  return(pwm);
  // Consider updating to include differential control
}


//////////////////////////////////////////////////////////

// These are the encoder interupt funcitons, they should NOT be edited

void indexLeftEncoderCount()
{
  leftEncoderCount++;
}
//////////////////////////////////////////////////////////
void indexRightEncoderCount()
{
  rightEncoderCount++;
}
