/* Lab 4: maze solving
   last updated: gkn 20240327
*/

#include <PinChangeInterrupt.h>
#include <HCSR04.h> // If using any Ultrasonic Distance Sensors, 
// This code assumes the HC-SR04 Library by gamegine, but many work

// TODO: Copy constants and definitions from Lab 3
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

#define FORWARD             0
#define LEFT               -75
#define RIGHT               85

#define pushButton 11

#define STOPPED 0
#define SPEED 1
#define RUN 2
#define SLOW 3
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// Define IR Distance sensor Pins
#define LeftIR A2
#define RightIR  A5

// If using any Ultrasonic - change pins for your needs
#define trig 4
#define echo 3

// if side sensor is Ultrasonic
//HCSR04 sideUS(trig, echo);
// if front sensor is Ultrasonic
HCSR04 frontUS(trig, echo);

// Define the distance tolerance that indicates a wall is present
#define wallTol 20 //cm

// Define the distance tolerance on the front that might cause a crash
#define crashTol 7 //cm

uint8_t currentState = 0;

// Define a score look up table to help determine move directions
uint8_t idealScores[5][5] {
    {1, 1, 1, 1, 1},
    {1, 2, 2, 2, 2},
    {1, 2, 3, 3, 3},
    {1, 2, 3, 4, 4},
    {1, 2, 3, 4, 5}
  };
int moves[50]; // Empty array of 50 moves, probably more than needed, just in case

int maze[50];
int distance[50];
uint8_t pos[2];

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
float Kp[] = {5.0, 4.75};
float Kd[] = {1, 1};

float maxSpeedA = 93.7;
float maxSpeedB = 112.4;

float cntCm = 0.955;
float DistancePerRev = 28;
float EncoderCountsPerRev = 24;
float l = 9;
float cmDegree = (2*3.5*l)/360;

float desiredMaxSpeed = 40; // Input desired speed in cm/s, convert later
int desiredDistance = 30; // Input desired test distance in cm

volatile unsigned int leftEncoderCount = 0;
volatile unsigned int rightEncoderCount = 0;

int PDdelay = 20;
unsigned long nextPDtime = 0;
int printDelay = 200;
unsigned long nextPrintTime = printDelay;
float rampFraction = 0.3;
struct velProfile profile;
uint8_t currentDirection = NORTH;

void setup() {
  //TODO: Include setup code for any pins other than motors or encoders
  pinMode(pushButton, INPUT_PULLUP);
  Serial.begin(9600);
  motor_setup();
  encoder_setup();
  profile = genVelProfile(desiredMaxSpeed, desiredDistance, rampFraction);
}


void loop(){
  while (digitalRead(pushButton) == 1){
    
    Serial.print(readLeftDist());
    Serial.print("\t");
    Serial.print(readRightDist());
    Serial.print("\t");
    Serial.println(readFrontDist());
    
  }; // wait for button push
  delay(50); // debounce input
  while (digitalRead(pushButton) == 0); // wait for button release
  delay(50); // debounce input


  explore();
  /*run_motor(A, 0);
  run_motor(B, 0);
  solve();
  while(true) { //Inifnite number of runs, so you don't have to re-explore everytime a mistake happens
    while (digitalRead(pushButton) == 1); // wait for button push
    delay(50); // debounce input
    while (digitalRead(pushButton) == 0); // wait for button release
    runMaze();
    run_motor(A, 0);
    run_motor(B, 0);
  }
 */
}


float readLeftDist() { 
  // If IR distance sensor
  float reading = analogRead(LeftIR);
  float voltage = (reading/1024)*5;
  float dist = 1/(0.038*voltage + 0.00005) - 0.24;// Equation from your calibration;
  // if Ultrasonic
  // float dist = frontUS.dist(); //(returns in cm)

  return dist;
}


float readRightDist() {
  // If IR distance sensor
  float reading = analogRead(RightIR);
  float voltage = (reading/1024)*5;
  float dist = 1/(0.07*voltage + 0.0045) - 0.24 ;// Equation from your calibration;
  // IF Ultrasonic
  // float dist = sideUS.dist(); //(returns in cm)

  return dist;
}

float readFrontDist() {
  // IF Ultrasonic
  float dist = frontUS.dist(); //(returns in cm)

  return dist;
}

void explore() {
  while (digitalRead(pushButton) == 1) { //while maze is not solved
    // Read distances
    static uint16_t lastcount = 0;
    static uint8_t ii = 0;
    static uint8_t jj = 0;
    static float rightSide;
    static float leftSide;
    static float front;
    static float avgTotRight;
    static float avgTotLeft;
    static float avgTotFront;
    static float rightRead[8];
    static float leftRead[8];
    static float frontRead[8];

    while(jj<8){ // capture sample for averaging sensor inputs
      rightRead[jj] = readRightDist();
      avgTotRight += rightRead[jj];
      leftRead[jj] = readLeftDist();
      avgTotLeft += leftRead[jj];
      frontRead[jj] = readFrontDist();
      avgTotFront += frontRead[jj];
      jj++;
    }
    if(jj+1 <16){ // Calculate average of sensor inputs 
      jj++;
      avgTotRight -= rightRead[jj-8];
      rightRead[jj-8] = readRightDist();
      avgTotRight += rightRead[jj-8];
      rightSide = avgTotRight/8;

      avgTotLeft -= leftRead[jj-8];
      leftRead[jj-8] = readLeftDist();
      avgTotLeft += leftRead[jj-8];
      leftSide = avgTotLeft/8;

      avgTotFront -= frontRead[jj-8];
      frontRead[jj-8] = readFrontDist();
      avgTotFront += frontRead[jj-8];
      front = avgTotFront/8;

    } else {
      jj = 8;
      avgTotRight -= rightRead[jj-8];
      rightRead[jj-8] = readRightDist();
      avgTotRight += rightRead[jj-8];
      rightSide = avgTotRight/8;

      avgTotLeft -= leftRead[jj-8];
      leftRead[jj-8] = readLeftDist();
      avgTotLeft += leftRead[jj-8];
      leftSide = avgTotLeft/8;

      avgTotFront -= frontRead[jj-8];
      frontRead[jj-8] = readFrontDist();
      avgTotFront += frontRead[jj-8];
      front = avgTotFront/8;
    }

    uint8_t desiredDirection;
    // If we are at a standstill and want to go forward
    if(currentState == STOPPED && desiredDirection == FORWARD)
    {
      driveForward(1); // Run acceleration profile
      
      currentState = RUN;
      Serial.println("Done Accelerating");
    } // If we are accelerated and can only go forwards
    else if(currentState == RUN && rightSide < wallTol && leftSide < wallTol && front > crashTol)
    {
      driveForward(2); // Continue to drive forwards
      if(leftEncoderCount>35+lastcount){
        Serial.print("moved one square");
        moves[ii] = 1;
        ii++;
        lastcount = leftEncoderCount;
        if(currentDirection == NORTH){
          pos[0]++;
        }else if(currentDirection == SOUTH){
          pos[0]--;
        }else if(currentDirection == EAST){
          pos[1]++;
        }else if(currentDirection == WEST){
          pos[1]--;
        }
      }
    }
    else if(rightSide>wallTol && leftSide<wallTol){
      run_motor(A, 0);
      run_motor(B, 0);
      uint16_t startingCounts = leftEncoderCount;
      if(currentDirection == NORTH && idealScores[pos[0], pos[1]+1] >idealScores[pos[0]+1, pos[1]]){
        run_motor(B, 0);
        turn(30, 110);
        run_motor(A, 0);
        run_motor(B, 0);
        jj = 0;
        currentDirection = EAST;
        pos[1]++;
      }else if(currentDirection == EAST && idealScores[pos[0]-1, pos[1]] >idealScores[pos[0], pos[1]+1]){
        run_motor(B, 0);
        turn(30, 110);
        currentDirection = SOUTH;
        pos[0]--;
      }else if(currentDirection == SOUTH && idealScores[pos[0], pos[1]-1] >idealScores[pos[0]-1, pos[1]]){
        run_motor(B, 0);
        turn(30, 110);
        currentDirection = WEST;
        pos[1]--;
      }else if(currentDirection == WEST && idealScores[pos[0]+1, pos[1]] >idealScores[pos[0], pos[1]-1]){
        run_motor(B, 0);
        turn(30, 110);
        currentDirection = NORTH;
        pos[0]++;
      }
      else{
        
        driveForward(1);
        if(leftEncoderCount-startingCounts > 30){
          if(currentDirection == NORTH){
            pos[0]++;
          }else if(currentDirection == SOUTH){
            pos[0]--;
          }else if(currentDirection == EAST){
            pos[1]++;
          }else if(currentDirection == WEST){
            pos[1]--;
          }
        }
      }
      
      if(currentDirection == NORTH){
        pos[0]++;
      }else if(currentDirection == SOUTH){
        pos[0]--;
      }else if(currentDirection == EAST){
        pos[1]++;
      }else if(currentDirection == WEST){
        pos[1]--;
      }
    }else if(rightSide>wallTol&&leftSide>wallTol&&front>crashTol){
      driveForward(1);
      run_motor(A, 0);
      run_motor(B, 0);
      jj=0;
    } else if (rightSide>wallTol&&leftSide>wallTol&&front<crashTol){
      if(currentDirection == NORTH&&idealScores[pos[0], pos[1]+1]>=idealScores[pos[0], pos[1]-1]){
        run_motor(B, 0);
        turn(30, 110);
        currentDirection = EAST;
        pos[0]++;
      }else{
        run_motor(B, 0);
        turn(30, -110);
        currentDirection = WEST;
        pos[0]++;
      }
    }
    else
    {
      currentState = SLOW;
      driveForward(3);
      while(true){
        Serial.print(rightSide);
        Serial.print("\t");
        Serial.print(leftSide);
        Serial.print("\t");
        Serial.print(front);
        Serial.print("\t");
        Serial.println(moves[0]);

      }
      currentState = STOPPED;
    }
  }
}



void solve() {
  // Write your own algorithm to solve the maze using the list of moves from explore
}


void runMaze() {
   int j = 0;
  // Wait for the push button to be pressed and released
  while (digitalRead(buttonPin) == 1); // Wait for button press
  delay(50); // Debounce delay
  while (digitalRead(buttonPin) == 0); // Wait for button release
  for (int i = 0; i < sizeof(maze)/2; i++) { 
    if(moves[i] == FORWARD){
      //driveForward(desiredMaxSpeed, distance[j]); // Drive forward for the specified distance
      j++;
    }
    else {
      run_motor(A, 0);
      turn(30, moves[i]); // Turn in the specified direction (left or right)
    }
    // Stop the motors after each move
    run_motor(A, 0);
    run_motor(B, 0);

  }
  j = 0; // Reset the distance index

}


// Copy any necessary functions from Lab 3
// Consider putting those functions in another .ino file that can be called from this one
// (Any ino files in a folder are automatically imported to the one that shares
// a name with the folder title)

void driveForward(int forwardState) {
// Accelerate - follow the trapiziodal profile generated at the beginning
  static float Xd = 0;
  static float Vd = 0;
  static float VA = 0;
  static float VB = 0;
  static int encA = 0;
  static int encB = 0;
  static unsigned long startTime = millis();
  static int cmdA = 0;
  static int cmdB = 0;
  // Run until final time of the velocity profile + 1 second, in order to
  // allow your motors to catch up if necessary
  if (forwardState == 1)
  {
    startTime = millis();
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    int prevEncA = 0;
    int prevEncB = 0;
    nextPDtime = 0;
    while(millis() - startTime < profile.t1*1000+100){
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
        Serial.println("cmdA");
        Serial.println(cmdA);
        // Update previous encoder counts
        prevEncA = encA;
        prevEncB = encB;

        // Set next time to update PD controller
        nextPDtime += PDdelay;
      }
    }
  }
  else if(forwardState == 2){
    // Run - run at max speed till told to do otherwise and try to keep the robot centered roughly
    static float leftDist = readLeftDist();
    static float rightDist = readRightDist();

    cmdA = 112;
    cmdB = 92;

    Serial.println("Driving");
    //float L = 4;
   // float wallError = leftDist-rightDist;
    //run_motor(A, cmdA-(int)(wallError*L));
    run_motor(A, cmdA);
    run_motor(B, cmdB);
    
  } 
  else if(forwardState == 3){
    // Decelerate - slow down to a stop    
    nextPDtime = 0;
    Serial.println("Beginning slowdown");
    
    // Reset encoder counts at the beginning of the movement.
    startTime = millis();
    //leftEncoderCount = 0;
    //rightEncoderCount = 0;
    int prevEncA = leftEncoderCount;
    int prevEncB = rightEncoderCount;
    struct state offsetState = targetState(profile.t2, profile);
    int distOffsetL = leftEncoderCount-offsetState.x;
    int distOffsetR = rightEncoderCount-offsetState.x;
    Serial.println(offsetState.x);
    nextPDtime = 0;
    while(millis() - startTime+profile.t2*1000 < profile.tf*1000){
      unsigned long now = millis() - startTime+profile.t2*1000;
      run_motor(A, cmdA);
      run_motor(B, cmdB);
      if (now > nextPDtime) {
        struct state desiredState = targetState(now/1000.0, profile);
        Xd = desiredState.x; // Desired position
        Vd = desiredState.v; // Desired speed
        // Get current encoder counts
        encA = leftEncoderCount-distOffsetL;
        encB = rightEncoderCount-distOffsetR;
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
    }
    run_motor(A, 0);
    run_motor(B, 0);
  }
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
  nextPDtime = 0;
  while (millis() - startTime < profile.tf*1000+300) {
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
      // Get current encoder counts
      encA = leftEncoderCount;
      encB = rightEncoderCount;
      // Get current speed (the 1000 converts PDdelay to seconds)
      VA = (encA - prevEncA) * 1000.0/PDdelay;
      VB = (encB - prevEncB) * 1000.0/PDdelay;
      // Feed-forward values of pwm for speed based on max speed
      float pwmInA = map(Vd, 0, maxSpeedA, 90, 255); 
      float pwmInB = map(Vd, 0, maxSpeedB, 70, 255);

      // Get command values from controller (as a byte)
      cmdA = pdController(pwmInA, Vd-VA, Xd-encA, 6, 0.3);
      cmdB = pdController(pwmInB, Vd-VB, Xd-encB, 6, 0.3);

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
  int cmd = constrain(u, -255, 255);
  return cmd;
}

