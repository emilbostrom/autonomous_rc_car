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
  const int minMotorSpeed = 600; // 0 to 1023
  const int maxMotorSpeed = 700; // out of 1023. But ESC does not want over 98%
  int motorDutyCycle;
  int direction;

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
    Serial.print("Received Motor Command: ");
    Serial.println(command);
  } 
  else if (messageType == SERVO_CONTROL_MSG) {
    // Do something with the servo angle
    Serial.print("Received Servo Reference: ");
    Serial.println(command);
  } 

  // variable for storing the potentiometer value
  int potentiometerValue = 0;
  // Reading potentiometer value
  potentiometerValue = analogRead(readPotentiometerPin);
  //Serial.println(potentiometerValue);
}