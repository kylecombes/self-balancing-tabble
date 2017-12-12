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
float currTime = millis() / 1000.0;

const float pi = 3.141529;

void setup() {
  Dynamixel.begin(SERVO_BAUDRATE);           // We now need to set Ardiuno to the new Baudrate speed
  Dynamixel.setDirectionPin(SERVO_ControlPin);   // Optional. Set direction control pin
  Dynamixel.setMode(SERVO_ID, SERVO, CW_LIMIT_ANGLE, CCW_LIMIT_ANGLE);
//  Serial.begin(9600);
}

float getDesiredPos(float tableAngle, float currTime){
  float deltaT;
  float angleDeriv;
  float Kp = 0.5 * 8;
  float Ki = 0.116;
  float Kd = 0.1;
  
  deltaT = currTime - prevTime;
  float angleErr = -tableAngle;
  angleIntegral = angleIntegral + angleErr * deltaT;
  float deltaAngle = (tableAngle - prevAngle);
  angleDeriv = deltaAngle / (deltaT);
//  Serial.println((String)deltaT + "\t" + (String)angleErr + "\t" + (String)deltaAngle + "\t" + (String)angleIntegral + "\t" + (String)angleDeriv);
  float kpm = angleErr * Kp;
  float kim = angleIntegral * Ki;
  float kdm = angleDeriv * Kd;
//  Serial.println("Kp product:  " + (String)kpm + "\tKi product:  " + (String)kim + "\tKd product:  " + (String)kdm);
  desiredPos = kpm + kim + kdm;
  prevAngle = tableAngle;
  return desiredPos;
}

#define ROUND_NEAREST 5

void setMotor(float desiredPos){
    int motorCommand;
    float posReltoMotor;
    
    if (abs(desiredPos) <= 1){
      desiredPos = desiredPos;
    }
    else if (desiredPos < -1) {
      desiredPos = -1;
    }
    else if (desiredPos > 1) {
      desiredPos = 1;
    }
      
    posReltoMotor = desiredPos + 0.5;     // Convert distance to motor frame
    motorCommand = (int) (posReltoMotor * 4095);
    //motorCommand = posReltoMotor / 0.36576;    // Distance in m to rotations in rad
    motorCommand = (motorCommand / ROUND_NEAREST) * ROUND_NEAREST;
//    Serial.println(motorCommand);
    //motorCommand = 0;
    Dynamixel.servo(SERVO_ID, motorCommand, 0x3FF);   // Move servo to potentiometer position
}

void loop() {
  currTime = millis() / 1000.0;
  potRead = analogRead(POT_PIN);    // read the value from the sensor

  // TODO: convert pot reading to table angle
  //       implement controller
  //       input table angle to control system

  tableAngle = map(potRead, 0.0, 1023.0, -100*pi/4, 100*pi/4);
  tableAngle = (tableAngle / 100.0) + .02;
  desiredPos = getDesiredPos(tableAngle, currTime);
  setMotor(desiredPos);
  prevTime = currTime;
  prevAngle = tableAngle;
//  Serial.print("Current table angle: ");
//  Serial.println(tableAngle);
//  Serial.print("Goal motor position: ");
//  Serial.println(desiredPos);
  delay(5); // delay for 1ms
}


