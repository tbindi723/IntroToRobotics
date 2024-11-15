// Define constants and pins
#define SHIELD false
#define buttonPin 12

#define EncoderMotorLeft 7

#define A 0
#define B 1

#define motorApwm 3
#define motorAdir 12
#define motorBpwm 11
#define motorBdir 13

#define LeftIR A0

// Driver Pin variable - any 4 analog pins (marked with ~ on your board)
#define IN1 9
#define IN2 10
#define IN3 5
#define IN4 6

// Define MM_PER_TICK
const double MM_PER_TICK = 0.955; // Millimeters per encoder tick; adjust as needed

// PID control variables
double Kp = 2.5, Ki = 1.0, Kd = 1.2;  // PID tuning parameters
double setpoint = 60.0;    // Desired tension in mm
double input = 0.0;        // Current tension
double output = 0.0;       // Motor output
double prevError = 0.0;    // Previous error
double integral = 0.0;     // Integral term accumulator
double outputMin = -255;   // Minimum motor speed
double outputMax = 255;    // Maximum motor speed

// External encoder variables (declared in the provided encoder file)
extern volatile unsigned int leftEncoderCount;

// Function prototypes
void computePID();
void setMotorSpeed(int pwmPin, int dirPin, double speed);
void stopMotor(int pwmPin, int dirPin);
void tensionMotorControl();
void launchSequence();
float readLeftIR();

void setup() {
  Serial.begin(9600);

  // Button setup
  pinMode(buttonPin, INPUT_PULLUP); 

  // Motor setup
  pinMode(motorApwm, OUTPUT);
  pinMode(motorAdir, OUTPUT);
  pinMode(motorBpwm, OUTPUT);
  pinMode(motorBdir, OUTPUT);

  // Encoder setup (called from the provided encoder file)
  encoder_setup(); 
}

void loop() {
  // Update the tension input based on encoder ticks
  input = leftEncoderCount * MM_PER_TICK;

  // Compute PID output
  computePID();

  // Apply PID output to control the motor
  tensionMotorControl();

  // Read the distance from the IR sensor
  //float distance = readLeftIR();
  //Serial.print("Distance (IR): ");
  //Serial.println(distance);

  // Check for button press to trigger the launch sequence
  if (digitalRead(buttonPin) == LOW) {
    delay(50);  // Debounce delay
    if ((digitalRead(buttonPin) == LOW) && (distance < 10)) { 
      launchSequence();
      //Serial.print("button works");
    }
  }
}

void computePID() {
  double error = setpoint - input;  // Calculate error
  integral += error;               // Accumulate integral term
  double derivative = error - prevError;  // Calculate derivative term
  output = Kp * error + Ki * integral + Kd * derivative;  // PID formula

  // Constrain output to the motor limits
  output = constrain(output, outputMin, outputMax);

  // Update the previous error
  prevError = error;
}

void tensionMotorControl() {
  if (output > 0) {
    setMotorSpeed(motorApwm, motorAdir, output);  // Motor A forward
    stopMotor(motorBpwm, motorBdir);             // Motor B stop
  } else if (output < 0) {
    setMotorSpeed(motorBpwm, motorBdir, -output);  // Motor B forward
    stopMotor(motorApwm, motorAdir);               // Motor A stop
  } else {
    stopMotor(motorApwm, motorAdir);  // Stop both motors
    stopMotor(motorBpwm, motorBdir);
  }
}

void setMotorSpeed(int pwmPin, int dirPin, double speed) {
  digitalWrite(dirPin, speed > 0 ? HIGH : LOW);  // Set motor direction
  analogWrite(pwmPin, abs(speed));              // Set motor speed
}

void stopMotor(int pwmPin, int dirPin) {
  analogWrite(pwmPin, 0);  // Stop PWM signal
  digitalWrite(dirPin, LOW);  // Reset direction pin
}

void launchSequence() {
  Serial.println("Launching...");

  // Move the catch motor to release
  setMotorSpeed(motorBpwm, motorBdir, 255); 
  delay(500);  // Adjust timing for release
  stopMotor(motorBpwm, motorBdir);  // Stop the catch motor

  // Reset encoder ticks for the next cycle
  leftEncoderCount = 0;
  Serial.println("Launch complete. Resetting...");
  delay(1000);  // Wait before accepting new commands
}

float readLeftIR() {
  float reading = analogRead(LeftIR);               // Read the IR sensor
  float voltage = (reading / 1023.0) * 5.0;         // Convert to voltage
  float dist = 1 / (0.038 * voltage + 0.00005) - 0.24;  // Calibration equation
  return dist;
}
