/*********************************************************************
    ROSArduinoBridge

    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org

    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson

    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

       Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
       Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
/* The Pololu VNH5019 dual motor driver shield */
//#define POLOLU_VNH5019

/* The Pololu MC33926 dual motor driver shield */
//#define POLOLU_MC33926

/* The RoboGaia encoder shield */
//#define ROBOGAIA

/* Encoders directly attached to Arduino board */
#define ARDUINO_ENC_COUNTER

/* L298 Motor driver*/
#define L298_MOTOR_DRIVER

#define MDD10A_MOTOR_DRIVER
#endif

#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
//#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
#include <Servo.h>
#include "servos.h"
#endif

#ifdef USE_BASE
/* Motor driver function definitions */
#include "motor_driver.h"

/* Encoder driver function definitions */
#include "encoder_driver.h"

/* PID parameters and functions */
#include "diff_controller.h"

/* Run the PID loop at 30 times per second */
#define PID_RATE           30     // Hz

/* Convert the rate into an interval */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Track the next time we make a PID calculation */
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
long AUTO_STOP_INTERVAL = 5000;
long lastMotorCommand = AUTO_STOP_INTERVAL;

long long int endEncoderLEFT = 0;
long long int endEncoderRIGHT = 0;
bool stateL = false;
bool stateR = false;
bool activeFunc = false;

#endif

/* Variable initialization */
long time_to_stopMoveX = 0;
long lastMotorCommandX = time_to_stopMoveX;
bool moveXActive = false;
// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;
int servoPipeline[5][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
int servoPipelineIndex = 0;
bool servoPipelineNext = true;
int stopLenght = 30;
bool urtraStop = false;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    case PING:
      Serial.println(Ping());
      break;
#ifdef USE_SERVOS
    case SERVO_WRITE:
      servos[arg1].setTargetPosition(arg2);
      Serial.println("OK");
      break;
    case SERVO_READ:
      Serial.println(servos[arg1].getServo().read());
      break;
#endif

#ifdef USE_BASE
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        resetPID();
        moving = 0;
      }
      else moving = 1;
      leftPID.TargetTicksPerFrame = arg1;
      rightPID.TargetTicksPerFrame = arg2;
      activeFunc = false;
      Serial.println("OK");
      break;
    case MOTOR_RAW_PWM:
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      resetPID();
      moving = 0; // Sneaky way to temporarily disable the PID
      setMotorSpeeds(arg1, arg2);
      activeFunc = false;
      Serial.println("OK");
      break;
    case MOVE_X:
      lastMotorCommandX = millis();
      resetPID();
      moving = 0;
      time_to_stopMoveX = arg2;
      moveXActive = true;
      moveX(arg1);
      activeFunc = false;
      Serial.println("OK");
      break;
    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != '\0') {
        pid_args[i] = atoi(str);
        i++;
      }
      Kp = pid_args[0];
      Kd = pid_args[1];
      Ki = pid_args[2];
      Ko = pid_args[3];
      Serial.println("OK");
      break;
    case TURN:
      lastMotorCommand = millis();
      resetPID();
      moving = 0;
      AUTO_STOP_INTERVAL = arg2;
      turn(arg1);
      activeFunc = false;
      Serial.println("OK");
      break;
    case FUNCTION:
      activeFunc = true;
      resetPID();
      resetEncoders();
      if (arg1 == 1) // หมุนขวา
      {
        //lastMotorCommand = millis();
        resetPID();
        moving = 0;
        endEncoderLEFT = 31969;
        endEncoderRIGHT = 30951;
        turn(1);
        //Serial.println("OK");
      }
      else if (arg1 == 2) // หมุนซ้าย
      {
        //lastMotorCommand = millis();
        resetPID();
        moving = 0;
        endEncoderLEFT = 31357;
        endEncoderRIGHT = 31559;
        turn(0);
        //Serial.println("OK");
      }
      else if (arg1 == 3) // หมุนกลับหลัง
      {
        //lastMotorCommand = millis();
        //        resetPID();
        //        moving = 0;
        //        endEncoderLEFT = 62714;
        //        endEncoderRIGHT = 63118;
        //        turn(0);
        turnF(1);
        turnF(1);
        Serial.println("OK");
        //Serial.println("OK");
      }
      else if (arg1 == 4) // เคลื่อนที่ไปทางซ้าย
      {
        lastMotorCommandX = millis();
        resetPID();
        moving = 0;
        time_to_stopMoveX = 250;
        moveXActive = true;
        moveX(80);
        activeFunc = false;
        //Serial.println("OK");
      }
      else if (arg1 == 5) // เคลื่อนที่ไปทางขวา
      {
        lastMotorCommandX = millis();
        resetPID();
        moving = 0;
        time_to_stopMoveX = 250;
        moveXActive = true;
        moveX(-80);
        activeFunc = false;
        //Serial.println("OK");
      }
      else if (arg1 == 6) // เคลื่อนที่ไปด้านหน้า ระบุระยะ
      {
        //lastMotorCommand = millis();
        resetPID();
        moving = 0;
        setMotorSpeeds(50, 50);
        endEncoderLEFT = 14900.0 / 31.41592 * arg2;
        endEncoderRIGHT = 14900.0 / 31.41592 * arg2;
        //Serial.println("OK");
      }
      else if (arg1 == 7) // เคลื่อนที่ไปด้านหลัง ระบุระยะ
      {
        //lastMotorCommand = millis();
        resetPID();
        moving = 0;
        setMotorSpeeds(-50, -50);
        endEncoderLEFT = 14900.0 / 31.41592 * arg2;
        endEncoderRIGHT = 14900.0 / 31.41592 * arg2;
        //Serial.println("OK");
      }
      else if (arg1 == 8) // servo จับ
      {
        servos[0].setTargetPosition(23);
        for (int i = 0 ; i < 67; i++) {
          servos[0].doSweep();
          delay(15);
        }
        delay(200);
        moveY(-17);
        delay(200);
        servos[1].setTargetPosition(0);
        for (int i = 0 ; i < 70; i++) {
          servos[1].doSweep();
          delay(25);
        }
        Serial.println("OK");
      }
      else if (arg1 == 9) // servo ปล่อย
      {
        servos[0].setTargetPosition(90);
        for (int i = 0 ; i < 67; i++) {
          servos[0].doSweep();
          delay(15);
        }
        delay(200);
        servos[1].setTargetPosition(70);
        for (int i = 0 ; i < 70; i++) {
          servos[1].doSweep();
          delay(25);
        }
        delay(200);
        moveY(-15);
        //Serial.println("OK");
      }
      else if (arg1 == 10) // เคลื่อนที่ไปด้านหน้าจนถึงระยะ ระบุระยะ
      {
        resetPID();
        moving = 0;
        setMotorSpeeds(70, 70);
        stopLenght = arg2;
        urtraStop = true;
        //Serial.println("OK");
      }
      else if (arg1 == 11) // startState1
      {
        activeFunc = false;
        if (arg2 == 0)
        {
          startState1();
        }
        else if (arg2 == 1)
        {
          continueState1();
        }
        else if (arg2 == 2)
        {
          backState1();
        }
        else if (arg2 == 3)
        {
          startState2();
        }
        else if (arg2 == 4)
        {
          continueState2();
        }
        else if (arg2 == 5)
        {
          backState2();
        }


        //Serial.println("OK");
      }
      else if (arg1 == 12)
      {
        moveY(arg2);
        //Serial.println("OK");
      }
      else if (arg1 == 13)
      {
        turnF(arg2);
        //Serial.println("OK");
      }
      else if(arg1 == 14) // เคลื่อนไปหัวจ่ายซ้าย
      {
        lastMotorCommandX = millis();
        resetPID();
        moving = 0;
        time_to_stopMoveX = arg2;
        moveXActive = true;
        moveX(150);
        activeFunc = false;
      }
       else if(arg1 == 15) // เคลื่อนไปหัวจ่ายขวา
      {
        lastMotorCommandX = millis();
        resetPID();
        moving = 0;
        time_to_stopMoveX = arg2;
        moveXActive = true;
        moveX(-150);
        activeFunc = false;
      }
      break;

#endif
    default:
      Serial.println("Invalid Command");
      break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

  // Initialize the motor controller if used */
#ifdef USE_BASE
#ifdef ARDUINO_ENC_COUNTER

  pinMode(ENCODER_LEFT_BACK, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_FRONT, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_BACK, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_FRONT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_BACK), updateENCODER_LEFT_BACK, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_FRONT), updateENCODER_LEFT_FRONT, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_BACK), updateENCODER_RIGHT_BACK, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_FRONT), updateENCODER_RIGHT_FRONT, RISING);
#endif
  initMotorController();
  resetPID();
#endif

  /* Attach servos if used */
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].initServo(
      servoPins[i],
      stepDelay[i],
      servoInitPosition[i]);
  }
#endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == '\n') {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

  // If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL && !moveXActive && !activeFunc && !urtraStop) {
    setMotorSpeeds(0, 0);
    moving = 0;
    //Serial.println("OK");
    //Serial.println("A");
  }

  if ((millis() - lastMotorCommandX) > time_to_stopMoveX && moveXActive && !activeFunc && !urtraStop) {
    moveXActive = false;
    setMotorSpeeds(0, 0);
    moving = 0;
    Serial.println("OK");
    //Serial.println("B");
  }
  if (endEncoderLEFT <= abs(readEncoder(LEFT)) && activeFunc)
  {
    stateL = true;
  }
  if (endEncoderRIGHT <= abs(readEncoder(RIGHT)) && activeFunc)
  {
    stateR = true;
  }
  if (activeFunc && stateL && stateR && !urtraStop)
  {
    Serial.println("OK");
    //Serial.println("C");
    activeFunc = false;
    stateL = false;
    stateR = false;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
  if (urtraStop)
  {
    if (stopLenght >= Ping())
    {
      //Serial.println("D");
      setMotorSpeeds(0, 0);
      delay(200);
      if (stopLenght < Ping())
      {
        setMotorSpeeds(70, 70);
      }
      else
      {
        urtraStop = false;
        Serial.println("OK");
      }
    }
    delay(100);
  }

#endif

  // Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}


void startState1()
{
  moveY(285);
  delay(200);
  turnF(1);
  delay(200);
  moveY(260);
  delay(200);
  Serial.println("OK");
}
void continueState1()
{
  turnF(1);
  delay(200);
  moveY(145);
  delay(200);
  turnF(0);
  delay(200);
  moveY(145); //สั้นไป
  delay(200);
  turnF(1);
  Serial.println("OK");
}

void backState1()
{
  turnF(0);
  delay(200);
  moveY(415); //สั้นไป
  delay(200);
  turnF(0);
  delay(200);
  moveY(156);
  Serial.println("OK");
}

void startState2()
{
  moveY(730);
  delay(200);
  turnF(1);
  delay(200);
  moveY(260);
  delay(200);
  Serial.println("OK");
}
void continueState2()
{
  turnF(1);
  delay(200);
  moveY(140);
  delay(200);
  turnF(0);
  delay(200);
  moveY(145);
  delay(200);
  turnF(1);
  Serial.println("OK");
}

void backState2()
{
  turnF(0);
  delay(200);
  moveY(395);
  delay(200);
  turnF(0);
  delay(200);
  moveY(520);
  Serial.println("OK");
}

void moveY(int distance) {
  int lastEncoder = 0;
  int nowSpeed = 30;
  moving = 0; 
  resetEncoders();
  resetPID();
  endEncoderLEFT = 14900.0 / 31.41592 * abs(distance);
  endEncoderRIGHT = 14900.0 / 31.41592 * abs(distance);
  if (distance < 0)
  {
//    for (int i = 40; i < 150; i += 10)
//    {
//      setMotorSpeeds(-i, -i);
//      delay(100);
//    }
    setMotorSpeeds(-150, -150);
  }
  else
  {
//    for (int i = 40; i < 150; i += 10)
//    {
//      setMotorSpeeds(i, i);
//      delay(100);
//    }
    setMotorSpeeds(150, 150);
  }
  stopMotor();
}

void turnF(int dir)
{
  moving = 0;
  resetEncoders();
  resetPID();
  if (dir == 0)
  {
    endEncoderLEFT = 31969;
    endEncoderRIGHT = 30951;
    turn(0);
  }
  else
  {
    endEncoderLEFT = 31357;
    endEncoderRIGHT = 31559;
    turn(1);
  }
  stopMotor();

}

int stopMotor() {
  while (1)
  {
    bool slow = false;
    int lastEncoder;
    int nowSpeed = 150;
//    if (endEncoderLEFT - abs(readEncoder(LEFT)) <= 15000 && !slow)
//    {
//      slow = true;
//      lastEncoder = abs(readEncoder(LEFT));
//    }
    if (endEncoderLEFT <= abs(readEncoder(LEFT)))
    {
      stateL = true;
    }
    if (endEncoderRIGHT <= abs(readEncoder(RIGHT)))
    {
      stateR = true;
    }
//    if (slow)
//    {
//      while (endEncoderLEFT > abs(readEncoder(LEFT)))
//      {
//        if (abs(readEncoder(LEFT)) - lastEncoder > 2000)
//        {
//          nowSpeed -= 15;
//          int leftSpeed = (readEncoder(LEFT) >= 0) ? nowSpeed : -nowSpeed;
//          int rightSpeed = (readEncoder(RIGHT) >= 0) ? nowSpeed : -nowSpeed;
//          lastEncoder = abs(readEncoder(LEFT));
//          setMotorSpeeds(leftSpeed, rightSpeed);
//          if (nowSpeed < 50) {
//            break;
//          }
//        }
//      }
//    }
    if (stateL && stateR)
    {
      stateL = false;
      stateR = false;
      setMotorSpeeds(0, 0);
      moving = 0;
      return 0;
    }
  }
}
