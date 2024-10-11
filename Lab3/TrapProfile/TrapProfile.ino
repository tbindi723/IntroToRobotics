// TODO: Import header code from previous lab code to set up your motors, buttons, pins, etc.


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
float Kp[] = {5.0, 5.0};
float Kd[] = {10.0, 10.0};

// TODO: Max speed of each motor in counts/sec
float maxSpeedA = ;
float maxSpeedB = ;


// TODO: Choose a reasonable speed for your robot to drive
float desiredMaxSpeed = 20; // Input desired speed in cm/s, convert later
int desiredDistance = 50; // Input desired test distance in cm

volatile unsigned int leftEncoderCount = 0;
volatile unsigned int rightEncoderCount = 0;

int PDdelay = 20;
unsigned long nextPDtime = 0;
int printDelay = 200;
unsigned long nextPrintTime = printDelay;



void setup() {
  //TODO: Include setup code for any pins other than motors or encoders
  Serial.begin(9600);
  motor_setup();
  encoder_setup();
}

void loop() {
  while (digitalRead(buttonPin) == true); // Halts program by iterating digitalRead until button is pressed
  driveForward(desiredMaxSpeed, desiredDistance); // Drives forward (see below)
}

void driveForward(float desiredSpeed, float desiredDistance) {
  // TODO: Convert desiredSpeed to motorSpeed and countsDesired
  float motorSpeed = ...; // Convert from cm/s to counts/s
  float countsDesired = ...; // Can use Lab 2B equation here

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
  struct velProfile profile = genVelProfile(...); 

  // Run until final time of the velocity profile + 1 second, in order to
  // allow your motors to catch up if necessary
  while (millis() - startTime < profile.tf*1000 + 1000) {
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
}

void turn(float maxAngularSpeed, float degrees) {
  // TODO: Modify forward drive function to convert it to turning
}


struct state targetState(float time, struct velProfile vp) {
  // TODO: Calculate the desired position and velocity at given sample time.
  struct state output;
  output.x = ; // output desired position
  output.v = ; // output desired velocity
  return output;
}

int pdController(float FF, float Verr, float Xerr, float kp, float kd) {
  // Implements Feed-forward + feedback control
  int u = FF + kp*Xerr + kd*Verr;
  int cmd = constrain(u, 0, 255);
  return cmd;
}

