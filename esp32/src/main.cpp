/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/
*********/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <iostream>
#include <string>
#include <sstream>
#include "driver/mcpwm.h"
#include <cstdlib>

#define MOTOR_CONTROL_MSG 'M' // Message char for motor
#define SERVO_CONTROL_MSG 'S' // Message char for servo

const int motorForwardPin = 4; 
const int motorReversePin = 16; 
const int motorEnablePin = 17;
const int motorPwmChannel = 0; // No physical connection

const int readPotentiometerPin = 34;
const int servoRightPin = 25; 
const int servoLeftPin = 26; 
const int servoEnablePin = 27;
const int servoPwmChannel = 4; // No physical connection

void setup() {

  Serial.begin(115200); // Baud rate
  Serial.println("ESP32 started");

  // DC motor pin and pwm settings
  const int motorPwmFreq = 30000; //Hz
  const int motoPwmrResolution = 10; // 0-1023
  int motorPwmDutyCycle = 0;
  pinMode(motorForwardPin, OUTPUT);
  pinMode(motorReversePin, OUTPUT);
  pinMode(motorEnablePin, OUTPUT);
  ledcSetup(motorPwmChannel, motorPwmFreq, motoPwmrResolution); // set up motor pwm
  ledcAttachPin(motorEnablePin, motorPwmChannel);

  // servo pin and pwm settings
  const int servoPwmFreq = 30000; //Hz
  const int servoPwmResolution = 10; // 0-1023
  int servoPwmDutyCycle = 0;
  pinMode(servoRightPin, OUTPUT);
  pinMode(servoLeftPin, OUTPUT);
  pinMode(servoEnablePin, OUTPUT);
  ledcSetup(servoPwmChannel, servoPwmFreq, servoPwmResolution); // set up PM pwm
  ledcAttachPin(servoEnablePin, servoPwmChannel);

  pinMode(readPotentiometerPin, INPUT);
}


void setMotorSpeed(float motorCommand) {
  const int minMotorSpeed = 550; // 0 to 1023
  const int maxMotorSpeed = 650; // out of 1023. But ESC does not want over 98%
  int motorDutyCycle;

  // Scale motorCommand to duty cycle
  if (abs(motorCommand) > 0.0) {
    // Scale the floating-point value to fit within the range 0 to maxMotorSpeed
    motorDutyCycle = int(abs(motorCommand) * (maxMotorSpeed - minMotorSpeed)) + minMotorSpeed;
  }
  else { // set to 0 if no value is requested
    motorDutyCycle = 0;
  }

  // Ensure the mapped value stays within the range 0 to maxMotorSpeed
  motorDutyCycle = constrain(motorDutyCycle, 0, maxMotorSpeed);

  // Set direction by enabling correct pin
  if (motorCommand > 0) {
    digitalWrite(motorForwardPin, HIGH);
    digitalWrite(motorReversePin, LOW);
  }
  else if (motorCommand < 0) {
    digitalWrite(motorForwardPin, LOW);
    digitalWrite(motorReversePin, HIGH);
  }
  else {
    digitalWrite(motorForwardPin, LOW);
    digitalWrite(motorReversePin, LOW);
  }

  ledcWrite(motorPwmChannel, motorDutyCycle); // Set new duty cycle
}


void setServoPosition(float servoCommand) {
  int motorDutyCycle;

  // Set direction by enabling correct pin
  if (servoCommand > 0) {
    digitalWrite(servoRightPin, HIGH);
    digitalWrite(servoLeftPin, LOW);
    Serial.println("Servo going right");
  }
  else if (servoCommand < 0) {
    digitalWrite(servoRightPin, LOW);
    digitalWrite(servoLeftPin, HIGH);
    Serial.println("Servo going left");
  }
  else {
    digitalWrite(servoRightPin, LOW);
    digitalWrite(servoLeftPin, LOW);
  }
  Serial.println(abs(servoCommand));
  
  ledcWrite(servoPwmChannel, abs(servoCommand)); // Set new duty cycle
}

float getServoError(float command) {

  // Middle 1460
  const int minPosition = 1180;
  const int maxPosition = 1530;

  // variable for storing the potentiometer value
  int potentiometerValue = 0;
  // Reading potentiometer value
  potentiometerValue = analogRead(readPotentiometerPin);

  Serial.println(abs(potentiometerValue));

  int m = 1355;
  float k = 175;

  float potentiometerValueScaled = constrain(float(potentiometerValue - m)/k,-1,1);

  float servoError = command - potentiometerValueScaled;

  // PID CONTROLLER
  float p = 800;
  return constrain(p * servoError,-1000,1000);
}

void loop() {
  
  // If no serial is received motor will turn off
  char messageType = MOTOR_CONTROL_MSG;
  float command = 0;

  if (Serial.available() > 0) {
    messageType = Serial.read(); // Read the message type
    command = Serial.parseFloat();
  }

  if (messageType == MOTOR_CONTROL_MSG) {
    // Motor control message
      // Read the motor speed
    // Do something with the motor speed
    setMotorSpeed(command);
    //Serial.print("Received Motor Command: ");
    //Serial.println(command);
  } 
  else if (messageType == SERVO_CONTROL_MSG) {
    // Do something with the servo angle
    //Serial.print("Received Servo Reference: ");
    //Serial.println(command);
    float servoCommand = getServoError(command);
    setServoPosition(servoCommand);
  } 
}