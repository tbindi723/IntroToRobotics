// TODO: Import header code from previous lab code to set up your motors, buttons, pins, etc.

#include <PinChangeInterrupt.h>


#define buttonPin 11

#define EncoderMotorLeft 7
#define EncoderMotorRight 8

#define A 0
#define B 1

// If you have a kit with the moto shield, set this to true
// If you have the Dual H-Bridge controller w/o the shield, set to false
#define SHIELD false
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
///////////////////////////////////////////////////////////////
// Two structures for use in this lab.
// The velocity profile structure contains the important time steps and 
// max velocity
struct velProfile {
  float t1;
  float t2;
  float tf;
  float vel;
};

// The "state" structure includes both position and velocity information
struct state {
  float x;
  float v;
};


///////////////////////////////////////////////////////////////
// TODO: Input gains for each motor from motor testing code below
float Kp[] = {6.0, 4.5};
float Kd[] = {0.3, 0.3};

// TODO: Max speed of each motor in counts/sec
float maxSpeedA = 93.7;
float maxSpeedB = 112.4;

float cntCm = 0.955;
float DistancePerRev = 25.132;
float EncoderCountsPerRev = 24;
float l = 11.75;
float cmDegree = (2*3.14*l)/360;

// TODO: Choose a reasonable speed for your robot to drive
float desiredMaxSpeed = 50; // Input desired speed in cm/s, convert later
int desiredDistance = 60; // Input desired test distance in cm

volatile unsigned int leftEncoderCount = 0;
volatile unsigned int rightEncoderCount = 0;

int PDdelay = 20;
unsigned long nextPDtime = 0;
int printDelay = 200;
unsigned long nextPrintTime = printDelay;
float rampFraction = 0.3;



void setup() {
  //TODO: Include setup code for any pins other than motors or encoders
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(9600);
  motor_setup();
  encoder_setup();
}

void loop() {
  while (digitalRead(buttonPin) == true); // Halts program by iterating digitalRead until button is pressed
  driveForward(desiredMaxSpeed, desiredDistance); // Drives forward (see below)
  turn(7, 90);
}

void driveForward(float desiredSpeed, float desiredDistance) {
  // TODO: Convert desiredSpeed to motorSpeed and countsDesired
  float motorSpeed = (desiredSpeed/DistancePerRev)*EncoderCountsPerRev; // Convert from cm/s to counts/s
  float countsDesired = (desiredDistance / DistancePerRev) * EncoderCountsPerRev; // Can use Lab 2B equation here

  float Xd, Vd, VA, VB;
  int encA, encB;

  // Reset encoder counts at the beginning of the movement.
  unsigned long startTime = millis();
  int cmdA = 0;
  int cmdB = 0;
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  int prevEncA = 0;
  int prevEncB = 0;
  
  //TODO: Give this inputs (see trapezoidal.ino)
  struct velProfile profile = genVelProfile(desiredMaxSpeed, desiredDistance, rampFraction); 

  // Run until final time of the velocity profile + 1 second, in order to
  // allow your motors to catch up if necessary
  while (millis() - startTime < profile.tf*1000+1000) {
    unsigned long now = millis() - startTime;
    run_motor(A, cmdA);
    run_motor(B, cmdB);
    if (now > nextPDtime) {
      struct state desiredState = targetState(now/1000.0, profile);
      Xd = desiredState.x; // Desired position
      Vd = desiredState.v; // Desired speed
      // Get current encoder counts
      encA = leftEncoderCount;
      encB = rightEncoderCount;
      // Get current speed (the 1000 converts PDdelay to seconds)
      VA = (encA - prevEncA) * 1000.0/PDdelay;
      VB = (encB - prevEncB) * 1000.0/PDdelay;
      // Feed-forward values of pwm for speed based on max speed
      float pwmInA = map(Vd, 0, maxSpeedA, 0, 255); 
      float pwmInB = map(Vd, 0, maxSpeedB, 0, 255);

      // Get command values from controller (as a byte)
      cmdA = pdController(pwmInA, Vd-VA, Xd-encA, Kp[0], Kd[0]);
      cmdB = pdController(pwmInB, Vd-VB, Xd-encB, Kp[1], Kd[1]);

      // Update previous encoder counts
      prevEncA = encA;
      prevEncB = encB;

      // Set next time to update PD controller
      nextPDtime += PDdelay;
    }

    if (now > nextPrintTime) {
      // Output: time, error, velocity error, and pwm for both motors
      Serial.println(now);
      Serial.print("Motor A: ");
      Serial.print(Xd - encA);
      Serial.print("\t");
      Serial.print(Vd - VA);
      Serial.print("\t");
      Serial.println(cmdA);
      Serial.print("Motor B: ");
      Serial.print(Xd - encB);
      Serial.print("\t");
      Serial.print(Vd - VB);
      Serial.print("\t");
      Serial.println(cmdB);
      nextPrintTime += printDelay;
    }
  }
  // Stop motors
  run_motor(A, 0);
  run_motor(B, 0);
  Serial.println("done driving");
}

void turn(float maxAngularSpeed, float degrees) {
    // TODO: Convert desiredSpeed to motorSpeed and countsDesired
  float motorSpeed = ((maxAngularSpeed)/DistancePerRev)*EncoderCountsPerRev; // Convert from cm/s to counts/s
  float wheelDesiredDistance = abs(degrees)*cmDegree;
  float countsDesired = (abs(degrees)*cmDegree / DistancePerRev) * EncoderCountsPerRev; // Can use Lab 2B equation here

  float Xd, Vd, VA, VB;
  int encA; 
  int encB;

  // Reset encoder counts at the beginning of the movement.
  unsigned long startTime = millis();
  int cmdA = 0;
  int cmdB = 0;
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  int prevEncA = 0;
  int prevEncB = 0;
  
  //TODO: Give this inputs (see trapezoidal.ino)
  struct velProfile profile = genVelProfile(maxAngularSpeed, wheelDesiredDistance, rampFraction); 
  Serial.println(profile.tf);
  Serial.println(wheelDesiredDistance);
  Serial.println(desiredMaxSpeed);
  // Run until final time of the velocity profile + 1 second, in order to
  // allow your motors to catch up if necessary
  while (millis() - startTime < profile.tf*1000+1000) {
    unsigned long now = millis() - startTime;
    if(degrees>0){
      run_motor(A, cmdA);
    }else{
      run_motor(B, cmdB);
    }
    
    
    if (now > nextPDtime) {
      struct state desiredState = targetState(now/1000.0, profile);
      Xd = desiredState.x; // Desired position
      Vd = desiredState.v; // Desired speed
      Serial.println(Xd);
      Serial.println(Vd);
      // Get current encoder counts
      encA = leftEncoderCount;
      encB = rightEncoderCount;
      // Get current speed (the 1000 converts PDdelay to seconds)
      VA = (encA - prevEncA) * 1000.0/PDdelay;
      VB = (encB - prevEncB) * 1000.0/PDdelay;
      // Feed-forward values of pwm for speed based on max speed
      float pwmInA = map(Vd, 0, maxSpeedA, 60, 255); 
      float pwmInB = map(Vd, 0, maxSpeedB, 60, 255);

      // Get command values from controller (as a byte)
      cmdA = pdController(pwmInA, Vd-VA, Xd-encA, Kp[0], Kd[0]);
      cmdB = pdController(pwmInB, Vd-VB, Xd-encB, Kp[1], Kd[1]);

      // Update previous encoder counts
      prevEncA = encA;
      prevEncB = encB;

      // Set next time to update PD controller
      nextPDtime += PDdelay;
    }

    if (now > nextPrintTime) {
      // Output: time, error, velocity error, and pwm for both motors
      Serial.println(now);
      Serial.print("Motor A: ");
      Serial.print(Xd - encA);
      Serial.print("\t");
      Serial.print(Vd - VA);
      Serial.print("\t");
      Serial.println(cmdA);
      Serial.print("Motor B: ");
      Serial.print(Xd - encB);
      Serial.print("\t");
      Serial.print(Vd - VB);
      Serial.print("\t");
      Serial.println(cmdB);
      nextPrintTime += printDelay;
    }
  }
  // Stop motors
  run_motor(A, 0);
  run_motor(B, 0);
  Serial.println("done turning");
  Serial.println(encA);
  Serial.println(encB);
}


struct state targetState(float time, struct velProfile vp) {
  // TODO: Calculate the desired position and velocity at given sample time.
  struct state output;
  if(time<vp.t1){
    output.x = 0.5*(vp.vel/vp.t1)*time*time ; // output desired position
    output.v = (vp.vel/vp.t1)*time; // output desired velocity
    return output;
  }else if(time<vp.t2){
    output.x =vp.vel*time-(0.5*(vp.vel/vp.t1)*vp.t1*vp.t1); // output desired position
    output.v = vp.vel; // output desired velocity
    return output;
  }else if(time<vp.tf){
    output.x = -0.5*(vp.vel/vp.t1)*(time-vp.tf)*(time-vp.tf)+((vp.vel/vp.t1)*vp.t1*vp.t1)+(vp.vel*(vp.t2-vp.t1));
    output.v = -((vp.vel/vp.t1)*(time-vp.t2))+vp.vel; // output desired velocity
    return output;
  }else{
    output.x = ((vp.vel/vp.t1)*vp.t1*vp.t1)+(vp.vel*(vp.t2-vp.t1));
    output.v = 0;
    return output;
  }
}

int pdController(float FF, float Verr, float Xerr, float kp, float kd) {
  // Implements Feed-forward + feedback control
  int u = FF + kp*Xerr + kd*Verr;
  int cmd = constrain(u, 0, 255);
  return cmd;
}

