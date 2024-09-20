
/* Dead Reckoning
  20230127 - gknave
*/

// Complete Tasks in Initial_Robot_Testing.ino first!!

/* Program TODO LIST
  1) Change the milliSecondsPerCM constant to the value you found in testing
  2) Change the milliSecondsPer90Deg constant to the value you found in testing
  3) Change the PWM of the forward function to the values you found in testing
  4) Write code for the button pause functionality
  5) Write your own turn function
*/

// If you have a kit with the moto shield, set this to true
// If you have the Dual H-Bridge controller w/o the shield, set to false
#define SHIELD false

// Defining these allows us to use letters in place of binary when
// controlling our motor(s)
#define A 0
#define B 1

//SHIELD Pin varables - cannot be changed
#define motorApwm 3
#define motorAdir 12
#define motorBpwm 11
#define motorBdir 13

// Dual H-Bridge motor controller pin variables - can be any 4 analog pins (marked with ~ on your board)
// Only used if SHIELD is false.
#define IN1 9
#define IN2 10
#define IN3 5
#define IN4 6


// Preprocessor Definitions
#define FORWARD    0
#define LEFT       1
#define RIGHT     -1
#define pushButton 11

// the following converts centimeters into milliseconds
#define milliSecondsPerCM    20  //CHANGE THIS ACCORDING TO YOUR BOT
#define milliSecondsPer90Deg 320 //CHANGE THIS ACCORDNING TO YOUR BOT
#define PWM_A 143 //CHANGE THIS TO GET YOUR BOT TO DRIVE STRAIGHT
#define PWM_B 100 //CHANGE THIS TO GET YOUR BOT TO DRIVE STRAIGHT

// the itemized list of moves for the robot as a 1D array
// this setup assumes that all the turns are 90 degrees and that all motions are pairs of drives and turns.
int moves[] = {180, 90, 60, 180, 100, 60, 100, 150};
int turns[] = {LEFT, RIGHT, RIGHT, RIGHT, LEFT, RIGHT, RIGHT, RIGHT};

void setup() {
  // set up the motor drive ports
  motor_setup();
  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT_PULLUP);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  int i, dist, dir;
  long time;

  //This for loop steps (or iterates) through the array 'moves'
  for (i = 0; i < sizeof(moves) / 2; i = i + 1) {

    /* TODO
       Write code here to make the program pause until
       the button has been pressed. Remember that getting a value
       from the board is digitalRead(pinNumber) and if statements
       need a double equals sign (==) for comparisons
    */
    while(digitalRead(pushButton));
    delay(250);
    //Forward Leg of each step
    Serial.print("Step #:");
    Serial.println(i);
    dist = moves[i];
    Serial.print("Forward for ");
    time = Forward(dist);
    Serial.print(time);
    Serial.println(" ms");
    delay(1000);

    //Turn Leg of each step
    Serial.print("Turn #:");
    Serial.println(i);
    dir = turns[i];
    if (dir == LEFT) {
      time = Turn(90);
      Serial.print("turning LEFT ");
      Serial.print(time);
      Serial.println(" ms");
    }
    else {
      time = Turn(-90);
      Serial.println("turning RIGHT ");

      Serial.print(time);
      Serial.println(" ms");
    } // end of else motions conditional
    delay(1000);

  } // end of for loop
  Serial.println("That's All Folks!");
  delay(1000);
  exit(i);
} // the end

//////////////////////////////////////////////
unsigned long Forward(int distance) {
  unsigned long t;
  t = distance * milliSecondsPerCM; //Time to keep motors on


  //To drive forward, motors go in the same direction
  run_motor(A, PWM_A); //change PWM to your calibrations
  run_motor(B, PWM_B); //change PWM to your calibrations
  delay(t);
  run_motor(A, 0);
  run_motor(B, 0);
  return (t);
}

//////////////////////////////////////////////
unsigned long Turn(int degrees) {
  unsigned long t;
  int sign = degrees / abs(degrees); //Find if left or right
  t = (abs(degrees) / 90) * milliSecondsPer90Deg; //Time to keep motors on
  if(sign==LEFT){
    run_motor(A, -PWM_A); //change PWM to your calibrations
    run_motor(B, PWM_B); //change PWM to your calibrations
    delay(t);
    run_motor(A, 0);
    run_motor(B, 0);
    return (t);
  }else if(sign == RIGHT){
    run_motor(A, PWM_A); //change PWM to your calibrations
    run_motor(B, -PWM_B); //change PWM to your calibrations
    delay(t);
    run_motor(A, 0);
    run_motor(B, 0);
    return (t);
  }
  // The run motor command takes in a PWM value from -255 (full reverse) to 255 (full forward)
  /* TODO
   * Using the Forward function as a guide,
   * Write commands in this Turn function to power the
   * motors in opposite directions for the calculated time
   * and then shut off
   */
}
