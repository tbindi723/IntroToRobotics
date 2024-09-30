/* Visual encoder testing code
  gkn 09102024
*/

// Include Libraries
// This is the PinChangeInterrupt package by NicoHood, 
//      available in the library search (Sketch->Include Libraries->Manage Libraries)
#include <PinChangeInterrupt.h>

// The digital pins we'll use for the encoders
#define EncoderMotorLeft  7
#define EncoderMotorRight 8

// Initialize encoder counts to 0
// These are volatile because they change during interrupt functions
volatile unsigned int leftEncoderCount = 0;
volatile unsigned int rightEncoderCount = 0;


void setup() {
  // Begin serial monitor
  Serial.begin(9600);
  // Initialize encoders
  encoder_setup(); // Defined in encoders.ino
}

// The encoder count variables change when the signal in that pin changes.
// The code runs an interrupt function whenever that happens.
// It stops whatever else is running to execute a simple function
// as soon as the change is detected.
// This is all set up in the attached encoders.ino.
void loop()
{
  int leftValue = digitalRead(EncoderMotorLeft);
  int rightValue = digitalRead(EncoderMotorRight);
  Serial.println("Current signal: ");
  Serial.print("Left: ");
  Serial.print(leftValue);    // Outputs current value of left encoder pin
  Serial.print("\t Right: ");
  Serial.println(rightValue); // Outputs current value of right encoder pin
  Serial.println("Encoder Counts: ");
  Serial.print("Left: ");
  Serial.print(leftEncoderCount);    // Changes value when the signal in pin EncoderMotorLeft changes
  Serial.print("\t Right: ");
  Serial.println(rightEncoderCount); // Changes value when the signal in pin EncoderMotorRight changes
  Serial.println();
  delay(2000);
}
