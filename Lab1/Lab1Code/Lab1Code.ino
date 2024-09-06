/* MEGN 441 Lab 1
*  0842020 mshapiro
*  Updated 01122024 nave
*
* TODO inserted wherever edits are needed.
*
*/

#define pushButton 2 // install a button with its output into Pin 2
#define adc0 A0 // measure analog DC (ADC) voltage in Pin A0
#define adc1 A1 // measure analog DC (ADC) voltage in Pin A1
#define ledPin 13 // Builtin LED is connected to Pin 13

// H-Bridge initialization
#define SHIELD false // If you have a kit with the moto shield, set SHIELD to true
// If you have the Dual H-Bridge controller w/o the shield, set SHIELD to false

#define A 0 // Use letters instead of numbers for motors.ino
#define B 1

// SHIELD Pin varables - cannot be changed
// Can be deleted if using Dual H-Bridge
#define motorApwm 3 
#define motorAdir 12
#define motorBpwm 11
#define motorBdir 13
// Driver Pin variables - any 4 pwm pins (marked with ~ on your board)
// Can be deleted if using SHIELD
#define IN1 9 
#define IN2 10
#define IN3 5
#define IN4 6

unsigned long starttime, stoptime, deltatime;
bool started = false; // State variable to track system start/stop
int R = 10; //TODO: Input shunt resister value (in Ohms)
float ADC0, ADC1;

void setup() { // the setup routine runs once when you press reset:

  // Initialize I/O pins
  pinMode(pushButton, INPUT_PULLUP);  // set the pushbutton's pin as a pull-up input:
  pinMode(ledPin, OUTPUT);  //set the LED pin as an output
  pinMode(adc0, INPUT); // set both ADC pins to input
  pinMode(adc1, INPUT);

  //use pre-built functions for motor setup (found in motors.ino)
  motor_setup();
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() { // the loop routine runs over and over again forever (or until reset button/loss of power):

  // read the pushButton pin:
  bool buttonState = digitalRead(pushButton);
  // TODO: 1. Set up logic to run trial
    //Useful to debug button hardware:
    //Serial.println(buttonState);

  if (!buttonState & !started){
    // On trial start:
    started = true; //Keep track of trial start
    // 2. Run motor (turn on LED if you wish)
    run_motor(A, 255); // motor A is on at full speed (pwm = 0-255)
    digitalWrite(ledPin, HIGH); // LED to indicate motor running
    // 3. Measure start time
    starttime = millis();
    Serial.println("start \t stop \t dt \t I \t V");
    Serial.println(starttime);
  }else if (!buttonState & started){
    // 4. Read ADC voltages
    ADC0 = analogRead(adc0) * 5.0/1023.0; // Convert 10-bit value (0-1023) to voltage (0-5 V)
    ADC1 = analogRead(adc1) * 5.0/1023.0;
    
  } else if (buttonState & started){
    // On trial finish:
    // 3. Read timers
    stoptime = millis(); //stop timer
    // 5. Stop motor
    run_motor(A, 0); //turn motor off
    digitalWrite(ledPin, LOW); // turn LED off
    // 6. Calculate current, motor voltage, deltatime
    float current = (ADC0-ADC1)/R; //TODO (Use V=IR across small resistor, convert to mA)
    float motorVoltage = ADC1; //TODO
    deltatime = (stoptime-starttime); //TODO
    // 7. Print results to serial monitor
    Serial.print(" \t ");
    Serial.print(stoptime); //print stop time
    Serial.print("\t ");
    Serial.print(deltatime);
    Serial.print("ms\t ");
    //Serial.print(current*1000);
    //Serial.print("mA\t ");
    Serial.print(motorVoltage);
    Serial.println("V");
    started = false; // Reset for next trial
  }

}
