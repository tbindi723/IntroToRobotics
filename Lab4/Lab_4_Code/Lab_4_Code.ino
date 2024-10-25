/* Lab 4: maze solving
   last updated: gkn 20240327
*/

#include <PinChangeInterrupt.h>
#include <HCSR04.h> // If using any Ultrasonic Distance Sensors, 
// This code assumes the HC-SR04 Library by gamegine, but many work




// TODO: Copy constants and definitions from Lab 3




// Define IR Distance sensor Pins
#define frontIR A0
#define sideIR  A1

// If using any Ultrasonic - change pins for your needs
#define trig 4
#define echo 5

// if side sensor is Ultrasonic
HCSR04 sideUS(trig, echo);
// if front sensor is Ultrasonic
//HCSR04 frontUS(trig, echo);

// Define the distance tolerance that indicates a wall is present
#define wallTol 3 //cm

int moves[50]; // Empty array of 50 moves, probably more than needed, just in case



void setup() {
  //TODO: Include setup code for any pins other than motors or encoders
  Serial.begin(9600);
  motor_setup();
  encoder_setup();
}




void loop()
{

  while (digitalRead(pushButton) == 1); // wait for button push
  while (digitalRead(pushButton) == 0); // wait for button release
  explore();
  run_motor(A, 0);
  run_motor(B, 0);
  solve();
  while True { //Inifnite number of runs, so you don't have to re-explore everytime a mistake happens
    while (digitalRead(pushButton) == 1); // wait for button push
    while (digitalRead(pushButton) == 0); // wait for button release
    runMaze();
    run_motor(A, 0);
    run_motor(B, 0);
  }
}


float readFrontDist() { 
  // If IR distance sensor
  int reading = analogRead(frontIR);
  float dist = // Equation from your calibration;

  // if Ultrasonic
  // float dist = frontUS.dist(); //(returns in cm)

  return dist;
}


float readSideDist() {
  // If IR distance sensor
  int reading = analogRead(sideIR);
  float dist = // Equation from your calibration;

  // IF Ultrasonic
  // float dist = sideUS.dist(); //(returns in cm)
  
  return dist;
}



void explore() {
  while (digitalRead(pushButton) == 1) { //while maze is not solved
    // Read distances
    float side = readSideDist();
    float front = readFrontDist();
    if (side > wallTol) {// If side is not a wall
      // turn and drive forward
      // Record actions
    }
    else if (front > wallTol) {// else if front is not a wall
      // drive forward
      // Record action
    } else {
      // turn away from side
      // Record action
    }
  }
}



void solve() {
  // Write your own algorithm to solve the maze using the list of moves from explore
}


void runMaze() {
  for(int i = 0; i < sizeof(maze)/2; i++){
    // Tell robot to run through the solved maze
  }
}


// Copy any necessary functions from Lab 3
// Consider putting those functions in another .ino file that can be called from this one
// (Any ino files in a folder are automatically imported to the one that shares
// a name with the folder title)
