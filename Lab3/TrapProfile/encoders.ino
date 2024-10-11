void encoder_setup() {
  Serial.print("Encoder Testing Program ");
  Serial.print("Now setting up the Left Encoder: Pin ");
  Serial.print(EncoderMotorLeft);
  Serial.println();

  pinMode(EncoderMotorLeft, INPUT_PULLUP); //set the pin to input
  // The following code sets up the PinChange Interrupt
  // Valid interrupt modes are: RISING, FALLING or CHANGE
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(EncoderMotorLeft), indexLeftEncoderCount, CHANGE);
  /////////////////////////////////////////////////
  Serial.print("Now setting up the Right Encoder: Pin ");
  Serial.print(EncoderMotorRight);
  Serial.println();
  pinMode(EncoderMotorRight, INPUT_PULLUP); //set the pin to input
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(EncoderMotorRight), indexRightEncoderCount, CHANGE);
}

//////////////////////////////////////////////////////////
// Interrupt functions for encoders //
//////////////////////////////////////////////////////////
void indexLeftEncoderCount()
{
  leftEncoderCount++;
}
//////////////////////////////////////////////////////////
void indexRightEncoderCount()
{
  rightEncoderCount++;
}