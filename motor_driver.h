/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

  // #define RIGHT_MOTOR_BACKWARD 5
  // #define LEFT_MOTOR_BACKWARD  6
  // #define RIGHT_MOTOR_FORWARD  9
  // #define LEFT_MOTOR_FORWARD   10
  // #define RIGHT_MOTOR_ENABLE 12
  // #define LEFT_MOTOR_ENABLE 13

//Power of motor.
#define pwmLEFT_FRONT 8
#define dirLEFT_FRONT 9
#define pwmLEFT_BACK 10
#define dirLEFT_BACK 11
#define pwmRIGHT_BACK 6
#define dirRIGHT_BACK 7
#define pwmRIGHT_FRONT 4
#define dirRIGHT_FRONT 5


void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void moveX(int spd);
void turn(int dir);
