// Encoder variables
volatile unsigned int leftEncoderCount = 0;
volatile unsigned int rightEncoderCount = 0;

// Define encoder pins (must be interrupt-capable pins)
#define EncoderMotorLeft 2
#define EncoderMotorRight 3

void encoder_setup() {
  Serial.println("Encoder Testing Program");

  // Left encoder setup
  Serial.print("Now setting up the Left Encoder: Pin ");
  Serial.println(EncoderMotorLeft);
  pinMode(EncoderMotorLeft, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(EncoderMotorLeft), indexLeftEncoderCount, CHANGE);

  // Right encoder setup
  Serial.print("Now setting up the Right Encoder: Pin ");
  Serial.println(EncoderMotorRight);
  pinMode(EncoderMotorRight, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(EncoderMotorRight), indexRightEncoderCount, CHANGE);
}

void indexLeftEncoderCount() {
  leftEncoderCount++;
}

void indexRightEncoderCount() {
  rightEncoderCount++;
}
