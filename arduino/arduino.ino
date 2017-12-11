#include "Dynamixel_Serial.h"       // Library needed to control Dynamixal servo

#define SERVO_ID 0x02               // Dynamixel Servo ID
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_BAUDRATE 1000000      // Baudrate for talking to Dynamixel
#define CW_LIMIT_ANGLE 0x001        // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW_LIMIT_ANGLE 0xFFF       // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode

#define POT_PIN A2    // Input pin for the potentiometer
int potRead;      // Potentiometer reading

float prevTime = 0;
float prevAngle = 0;
float tableAngle;
float desiredPos;
float angleIntegral = 0;
int currTime = 0;

void setup() {
  Dynamixel.begin(SERVO_BAUDRATE);           // We now need to set Ardiuno to the new Baudrate speed
  Dynamixel.setDirectionPin(SERVO_ControlPin);   // Optional. Set direction control pin
  Dynamixel.setMode(SERVO_ID, SERVO, CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE);
}

float getDesiredPos(float tableAngle, float prevAngle, float currTime, float prevTime){
  float deltaT;
  float angleDeriv;
  float angleDiff;
  float Kp = 00190064;
  float Ki = 0.0000593951;
  float Kd = 0.0228077;
  
  deltaT = currTime - prevTime;
  angleDiff = tableAngle * -1;
  angleIntegral = angleIntegral + angleDiff * deltaT; 
  angleDeriv = (angleDiff - prevAngle) / (deltaT);
  desiredPos = angleDiff * Kp + angleIntegral * Ki + angleDeriv * Kd;
  return desiredPos;
}
void setMotor(float desiredPos){
    float motorCommand;
    float posReltoMotor;
    
    posReltoMotor = desiredPos - 0.5;     // Convert distance to motor frame
    motorCommand = posReltoMotor / 0.36576;    // Distance in m to rotations in rad
    Dynamixel.servo(SERVO_ID, motorCommand, 0x3FF);   // Move servo to potentiometer position
}

void loop() {
  currTime = millis();
  potRead = analogRead(POT_PIN);    // read the value from the sensor

  // TODO: convert pot reading to table angle
  //       implement controller
  //       input table angle to control system

  tableAngle = potRead * 5;
  desiredPos = getDesiredPos(tableAngle, prevAngle, currTime, prevTime);
  setMotor(desiredPos);
  prevTime = currTime;
  prevAngle = tableAngle;
  
}

