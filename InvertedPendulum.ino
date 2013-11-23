/****************************************
 * Controlled Inverted Pendulum         *
 *                                      *
 * Josh Vazquez 2013                    *
 *                                      *
 * Based on Arduino Starter Kit         *
 *  Project 10: Zoetrope                *
 * PID library by Brett Beauregard      *
 *                                      *
 ****************************************/

#include <PID_v1.h>

// PHYSICAL PINS *********************************************************************
// these are used to tell the IC which way to turn the motor
const int directionPin1 = 2;
const int directionPin2 = 3;

// where to send 0-255 signal for motor voltage 0-100%
const int pwmPin = 9;

// rotary encoder attachment
const int encoderPin = A0;

// CONSTANTS *************************************************************************
// encoder angle at top of arc
const int desiredAngle = 0;

// VARIABLES *************************************************************************
int motorDirection = 1;

// encoder sends raw values 0-1023
int encoderPosition = 0;

// degrees 0-359
float encoderAngle = 0;

// -179 <= relativeAngle <= 180: pendulum angle difference from top of arc
float relativeAngle = 0;

// difference between real angle and desired angle
float error = 0;

// PID VARIABLES *********************************************************************
double pidSetpoint, pidInput, pidOutput;
double realOutput;

// create the PID controller
//PID myPID(&pidInput, &pidOutput, &pidSetpoint, 2, 5, 1, DIRECT); // standard tuning
PID myPID(&pidInput, &pidOutput, &pidSetpoint, 4, 4, 0, DIRECT); // debug tuning

/*
// frontend
double Setpoint = desiredAngle;
double Input = encoderPin;
double Output = pwmPin;

unsigned long serialTime;
*/

// reverse the motor direction
void changeDirection() {
  if (motorDirection == 1) {
    digitalWrite(directionPin1, HIGH);
    digitalWrite(directionPin2, LOW);
  }
  else {
    digitalWrite(directionPin1, LOW);
    digitalWrite(directionPin2, HIGH);
  }
}

void setup() {
  // specify output pins being used
  pinMode(directionPin1, OUTPUT);
  pinMode(directionPin2, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  
  // start with 0 motor power
  digitalWrite(pwmPin, LOW);
  
  // PID variables
  pidInput = 0;
  pidSetpoint = desiredAngle;
  
  // turn PID on
  myPID.SetMode(AUTOMATIC);
  
  /* limits: fixes "0" returned for angles 0-180. Uses -128.0~128.0 so that the
   *  values can be boosted by +/-127 to give a motor output range of -255~-127,
   *  0, and 127~255. This is because the lower half of 0-255 gives too little
   *  voltage to the motor for it to spin. */
  myPID.SetOutputLimits(-128.0, 128.0);
  
  // get ready to log data to computer
  Serial.begin(9600);
}

void loop() {
  delay(1);
  
  // read encoder and compute angle
  encoderPosition = analogRead(encoderPin);
  
  // translate 0-1023 from encoder -> 0-359 degrees
  encoderAngle = encoderPosition / 1024.0 * 360.0;
  
  // with 0 degrees being the top and increasing clockwise, the left half of the
  //  rotation range is considered to be -179~-1 instead of 181~359.
  if (encoderAngle > 180) {
    relativeAngle = encoderAngle - 360;
  }
  else {
    relativeAngle = encoderAngle;
  }
  
  // difference from desiredAngle, doesn't account for desired angles other than 0
  error = relativeAngle;
  
  // do the PID magic
  pidInput = error;
  myPID.Compute();
  
  if (pidOutput < 0) {
    motorDirection = 0;
    changeDirection();
  }
  else {
    motorDirection = 1;
    changeDirection();
  }
  
  // minimum motor power is 50%
  if (pidOutput == 0) {
    realOutput = 0;
  }
  else if (pidOutput > 0) {
    realOutput = pidOutput + 127;
  }
  else if (pidOutput < 0) {
    realOutput = pidOutput - 127;
  }
  
  // if the pendulum is too far off of vertical to recover, turn off the PID and motor
  if (error > 45 || error < -45) {
    Serial.print("-- Angle out of bounds -- ");
    myPID.SetMode(MANUAL);
    analogWrite(pwmPin, 0);
    pidOutput = 0;
    realOutput = 0;
  }
  else {
    myPID.SetMode(AUTOMATIC);
    // in bounds
    Serial.print("-- MOTOR ACTIVE, write ");
    Serial.print(realOutput);
    Serial.print(" --");
    analogWrite(pwmPin, abs(realOutput));
  }
  
/*  
  // frontend
  if (millis()>serialTime) {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
 */
  
  
  // debug logging
  Serial.print("Relative angle: ");
  Serial.print(relativeAngle, 2);
  Serial.print(" -- PID output: ");
  Serial.println(pidOutput);
  //Serial.print(" -- proportional: ");
  //Serial.print(-- integral: ## -- derivative: ##")
  delay(100);
  
}



