/***************************************************************
   Motor driver definitions

   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   *************************************************************/

#ifdef USE_BASE

#ifdef MDD10A_MOTOR_DRIVER

#include "encoder_driver.h"
//#include "StateDir.h"
//StateDir stateMotor;
void initMotorController()
{
  pinMode(pwmLEFT_BACK, OUTPUT);
  pinMode(dirLEFT_BACK, OUTPUT);
  pinMode(pwmLEFT_FRONT, OUTPUT);
  pinMode(dirLEFT_FRONT, OUTPUT);
  pinMode(pwmRIGHT_BACK, OUTPUT);
  pinMode(dirRIGHT_BACK, OUTPUT);
  pinMode(pwmRIGHT_FRONT, OUTPUT);
  pinMode(dirRIGHT_FRONT, OUTPUT);
}
void setMotorSpeed(int i, int spd)
{
  if (i == LEFT) {
    //drive.setM1Speed(spd);
    digitalWrite(dirLEFT_FRONT, 1);
    digitalWrite(dirLEFT_BACK, 1);
    if (spd < 0)
    {
      digitalWrite(dirLEFT_FRONT, 0);
      digitalWrite(dirLEFT_BACK, 0);
    }
    analogWrite(pwmLEFT_FRONT, abs(spd) );
    analogWrite(pwmLEFT_BACK, abs(spd) );
    setDir(0,spd);
    setDir(1,spd);
  }
  else if (i == RIGHT)
  {
    //drive.setM2Speed(spd);
    digitalWrite(dirRIGHT_FRONT, 1);
    digitalWrite(dirRIGHT_BACK, 1);
    if (spd < 0)
    {
      digitalWrite(dirRIGHT_FRONT, 0);
      digitalWrite(dirRIGHT_BACK, 0);
    }
    analogWrite(pwmRIGHT_FRONT, abs(spd));
    analogWrite(pwmRIGHT_BACK, abs(spd));
    setDir(2,spd);
    setDir(3,spd);
  }
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int LeftSpeed, int RightSpeed)
{
  setMotorSpeed(LEFT, LeftSpeed);
  setMotorSpeed(RIGHT, RightSpeed);
}

void moveX(int spd)
{
    digitalWrite(dirLEFT_FRONT, 0);
    digitalWrite(dirLEFT_BACK, 1);
    digitalWrite(dirRIGHT_FRONT, 1);
    digitalWrite(dirRIGHT_BACK, 0);
    if (spd < 0)
    {
      digitalWrite(dirLEFT_FRONT, 1);
      digitalWrite(dirLEFT_BACK, 0);
      digitalWrite(dirRIGHT_FRONT, 0);
      digitalWrite(dirRIGHT_BACK, 1);
    }
    analogWrite(pwmLEFT_FRONT, abs(spd) );
    analogWrite(pwmLEFT_BACK, abs(spd) );
    analogWrite(pwmRIGHT_FRONT, abs(spd));
    analogWrite(pwmRIGHT_BACK, abs(spd));
}

void turn(int dir)
{
  if(dir == 0)
  {
//    for (int i = 40; i < 150; i += 10)
//    {
//      setMotorSpeeds(-i, i);
//      delay(100);
//    }
    setMotorSpeeds(-150, 150);
  }
  else
  {
//    for (int i = 40; i < 150; i += 10)
//    {
//      setMotorSpeeds(i, -i);
//      delay(100);
//    }
    setMotorSpeeds(150, -150);
  }
}


#elif POLOLU_VNH5019
/* Include the Pololu library */
#include "DualVNH5019MotorShield.h"

/* Create the motor driver object */
DualVNH5019MotorShield drive;

/* Wrap the motor driver initialization */
void initMotorController() {
  drive.init();
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) drive.setM1Speed(spd);
  else drive.setM2Speed(spd);
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#elif defined POLOLU_MC33926
/* Include the Pololu library */
#include "DualMC33926MotorShield.h"

/* Create the motor driver object */
DualMC33926MotorShield drive;

/* Wrap the motor driver initialization */
void initMotorController() {
  drive.init();
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) drive.setM1Speed(spd);
  else drive.setM2Speed(spd);
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#elif defined L298_MOTOR_DRIVER
void initMotorController() {
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
}

void setMotorSpeed(int i, int spd) {
  unsigned char reverse = 0;

  if (spd < 0)
  {
    spd = -spd;
    reverse = 1;
  }
  if (spd > 255)
    spd = 255;

  if (i == LEFT) {
    if      (reverse == 0) {
      analogWrite(LEFT_MOTOR_FORWARD, spd);
      analogWrite(LEFT_MOTOR_BACKWARD, 0);
    }
    else if (reverse == 1) {
      analogWrite(LEFT_MOTOR_BACKWARD, spd);
      analogWrite(LEFT_MOTOR_FORWARD, 0);
    }
  }
  else { /*if (i == RIGHT) //no need for condition*/
    if      (reverse == 0) {
      analogWrite(RIGHT_MOTOR_FORWARD, spd);
      analogWrite(RIGHT_MOTOR_BACKWARD, 0);
    }
    else if (reverse == 1) {
      analogWrite(RIGHT_MOTOR_BACKWARD, spd);
      analogWrite(RIGHT_MOTOR_FORWARD, 0);
    }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#else
#error A motor driver must be selected!
#endif

#endif
