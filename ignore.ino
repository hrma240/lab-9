//the right motor will be controlled by the motor A pins on the motor driver
#include <PID_v2.h> 

const int AIN1 = 13;           //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;            //control pin 2 on the motor driver for the right motor
const int PWMA = 11;            //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;           //speed control pin on the motor driver for the left motor
const int BIN2 = 9;           //control pin 2 on the motor driver for the left motor
const int BIN1 = 8;           //control pin 1 on the motor driver for the left motor

const int trigPin = 6;        //trigger pin for distance snesor
const int echoPin = 7;        //echo pin for distance sensor
const int RED = 3;
const int GREEN = 2;

String botDirection;           //the direction that the robot will drive in (this change which direction the two motors spin in)
String motorSpeedStr;

int motorSpeed;               //speed integer for the motors
float duration, distance;     //duration and distance for the distance sensor


double MEAS = 0;
double OUT = 0;
double setpoint = 15;
double Kp = 10;
double Ki = 1;
double Kd = 1;
PID myPID(&MEAS, &OUT, &setpoint, Kp, Ki, Kd, DIRECT); //new



void setup() 
  {
  //set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  //set the distance sensor trigger pin as output and the echo pin as input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(RED,OUTPUT);
  pinMode(GREEN,OUTPUT);

  Serial.begin(9600);           //begin serial communication with the computer

  //prompt the user to enter a command
  Serial.println("Enter a direction followed by speed.");
  Serial.println("f = forward, b = backward, r = turn right, l = turn left, s = stop");
  Serial.println("Example command: f 50 or s 0");

  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(AUTOMATIC);
  //NEW
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*340/10000)/2; // Units are cm
//  Serial.print("Distance: ");
//  Serial.println(distance);
    delayMicroseconds(50);
    myPID.Compute();// new
    
    motorSpeed = map(OUT, 0, 225, 150, 255);
    MEAS = distance;
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(MEAS);
    Serial.print(",");
    Serial.println(OUT);
    

  if (Serial.available() > 0)                         //if the user has sent a command to the RedBoard
  {
    botDirection = Serial.readStringUntil(' ');       //read the characters in the command until you reach the first space
    motorSpeedStr = Serial.readStringUntil(' ');           //read the characters in the command until you reach the second space
    motorSpeed = motorSpeedStr.toInt();
  }
  if (true)
  {   
                                                      //if the switch is in the ON position
   
    if (MEAS > setpoint)    
     { 
      botDirection = "f";
     }
    else
    {
      botDirection = "b";
    }
    if (botDirection == "f")                         //if the entered direction is forward
    {
     rightMotor(-motorSpeed);                                //drive the right wheel forward
      leftMotor(motorSpeed);                                  //drive the left wheel forward
      digitalWrite(GREEN,HIGH);
      digitalWrite(RED,LOW);
    }
    else if (botDirection == "b")                    //if the entered direction is backward
    {
      rightMotor(motorSpeed);                               //drive the right wheel forward
      leftMotor(-motorSpeed);                                //drive the left wheel forward
      digitalWrite(GREEN,HIGH);
      digitalWrite(RED,LOW);
    }
    else if (botDirection == "r")                     //if the entered direction is right
    {
      rightMotor(motorSpeed);                               //drive the right wheel forward
      leftMotor(motorSpeed);                                 //drive the left wheel forward
      digitalWrite(GREEN,HIGH);
      digitalWrite(RED,LOW);
    }
    else if (botDirection == "l")                   //if the entered direction is left
    {
      rightMotor(-motorSpeed);                                //drive the right wheel forward
      leftMotor(-motorSpeed);                                //drive the left wheel forward
      digitalWrite(GREEN,HIGH);
      digitalWrite(RED,LOW);
    }
    else if (botDirection == "s")
    {
      rightMotor(0);
      leftMotor(0);
      digitalWrite(GREEN,LOW);
      digitalWrite(RED,HIGH);
    }
  }
  else if (distance < 20)
  {
    if (botDirection == "b")
    {
      rightMotor(motorSpeed);                               //drive the right wheel forward
      leftMotor(-motorSpeed);                                //drive the left wheel forward
    }
    else
    {
      Serial.print("Object Detected at ");
      Serial.print(distance);
      Serial.println(" cm");
      rightMotor(0);                                  //turn the right motor off
      leftMotor(0);                                   //turn the left motor off
    }
  }
}

void rightMotor(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMA, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void leftMotor(int motorSpeed)                        //function for driving the left motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMB, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}
