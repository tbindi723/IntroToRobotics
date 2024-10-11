#include <PinChangeInterrupt.h>

// The "state" structure includes both position and velocity information
struct state {
  float x;
  float v;
};

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

volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;
volatile int dir = 1;

// TODO: Max speed of each motor in counts/sec
float maxSpeedA = 93.7;
float maxSpeedB = 112.4;

// TODO: Set a desired speed (in counts/second)
float Vdesired = 80;

// TODO: Choose gains
float Kp[] = {1.0, 1.0};
float Kd[] = {1.0, 1.0};


int PDdelay = 20;
unsigned long nextPDtime = 0;
int printDelay = 100;
unsigned long nextPrintTime = 0;


void setup() {
  // TODO: Initialize buttonPin as an INPUT_PULLUP
  pinMode(buttonPin, INPUT_PULLUP);
  motor_setup();
  encoder_setup();
  Serial.begin(9600);
  // Headers for data output to PLX-DAQ
  Serial.print("LABEL, Time, Target, Left Encoder Counts, Right Encoder Counts, ");
  Serial.print("Proportional Left, Differential Left, ");
  Serial.println("Proportional Right, Differential Right,");
}

void loop() {
  while (digitalRead(buttonPin) == true); // Halts program by iterating digitalRead until button is pressed
  
  // Reset encoder counts at the beginning of the movement.
  float Xd, Vd, VA, VB;
  int encA, encB;

  unsigned long startTime = millis();
  int cmdA = 0;
  int cmdB = 0;
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  int prevEncA = 0;
  int prevEncB = 0;

  while (millis() - startTime < 3000) {    
    unsigned long now = millis() - startTime;
    run_motor(A, cmdA);
    run_motor(B, cmdB);
    if (now > nextPDtime) { // We only update the controller perdiodically
      // Uses the targetState function below to identify the target position and velocity
      // These will be used to calculate our errors.
      struct state currentTarget = targetState(now/1000.0, Vdesired);
      Xd = currentTarget.x; // Desired position
      Vd = currentTarget.v; // Desired speed

      // Feed forward terms choose the PWM that your motor WOULD choose
      // if speed linearly depended on PWM
      // This is an open loop term
      float feedForwardA = map(Vd, 0, maxSpeedA, 0, 255); 
      float feedForwardB = map(Vd, 0, maxSpeedB, 0, 255);

      // Save the current encoder count, to keep all calculations based on the same measurement
      encA = leftEncoderCount;
      encB = rightEncoderCount;

      // Calculate the velocity based on the difference between current and previous
      // encoder counts. The 1000 converts milliseconds to seconds.
      VA = (encA - prevEncA) * 1000.0/PDdelay;
      VB = (encB - prevEncB) * 1000.0/PDdelay;

      // Update the previous encoder counts
      prevEncA = encA;
      prevEncB = encB;

      cmdA = pdController(feedForwardA, Vd-VA, Xd-encA, Kp[0], Kd[0]);
      cmdB = pdController(feedForwardB, Vd-VB, Xd-encB, Kp[1], Kd[1]);
    }
    if (now > nextPrintTime) {
      Serial.print("DATA, TIME, ");
      Serial.print(Xd);
      Serial.print(", ");
      Serial.print(encA);
      Serial.print(", ");
      Serial.print(encB);
      Serial.print(", ");
      Serial.print(Kp[0]*(Xd-encA));
      Serial.print(", ");
      Serial.print(Kd[0]*(Vd-VA));
      Serial.print(", ");
      Serial.print(Kp[1]*(Xd-encB));
      Serial.print(", ");
      Serial.print(Kd[1]*(Vd-VA));
      Serial.print(", ");
      Serial.println();
    }
  }
  run_motor(A, 0);
  run_motor(B, 0);
}

struct state targetState(float time, float Vdesired) {
  // TODO: Calculate the desired position and velocity at given sample time.
  struct state output;
  output.x = Vdesired*time; // output desired position
  output.v = Vdesired; // output desired velocity
  return output;
}

int pdController(float FF, float Verr, float Xerr, float kp, float kd) {
  // Implements Feed-forward + feedback control
  int u = FF + kp*Xerr + kd*Verr;
  int cmd = constrain(u, 0, 255);
  return cmd;
}