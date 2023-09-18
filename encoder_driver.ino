/* *************************************************************
   Encoder definitions

   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   ************************************************************ */

#ifdef USE_BASE

#ifdef ROBOGAIA
/* The Robogaia Mega Encoder shield */
#include "MegaEncoderCounter.h"

/* Create the encoder shield object */
MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode

/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == LEFT) return encoders.YAxisGetCount();
  else return encoders.XAxisGetCount();
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT) return encoders.YAxisReset();
  else return encoders.XAxisReset();
}
#elif defined(ARDUINO_ENC_COUNTER)
volatile long encoderCountLEFT_FRONT = 0;
volatile long encoderCountLEFT_BACK = 0;
volatile long encoderCountRIGHT_FRONT = 0;
volatile long encoderCountRIGHT_BACK = 0;
int State_dirLEFT_FRONT = 1;
int State_dirLEFT_BACK = 1;
int State_dirRIGHT_FRONT = 1;
int State_dirRIGHT_BACK = 1;
//#include "StateDir.h"
//  volatile long left_enc_pos = 0L;
//  volatile long right_enc_pos = 0L;
//  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
//
//  /* Interrupt routine for LEFT encoder, taking care of actual counting */
//  ISR (PCINT2_vect){
//  	static uint8_t enc_last=0;
//
//	enc_last <<=2; //shift previous state two places
//	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
//
//  	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
//  }
//
//  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
//  ISR (PCINT1_vect){
//        static uint8_t enc_last=0;
//
//	enc_last <<=2; //shift previous state two places
//	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
//
//  	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
//  }
void updateENCODER_LEFT_BACK() {
  encoderCountLEFT_BACK+=State_dirLEFT_BACK;
}
void updateENCODER_LEFT_FRONT() {
  encoderCountLEFT_FRONT+=State_dirLEFT_FRONT;
}
void updateENCODER_RIGHT_BACK() {
  encoderCountRIGHT_BACK+=State_dirRIGHT_BACK;
}
void updateENCODER_RIGHT_FRONT() {
  encoderCountRIGHT_FRONT+=State_dirRIGHT_FRONT;
}
/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == LEFT) return encoderCountLEFT_FRONT;
  else return encoderCountRIGHT_FRONT;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT) {
    encoderCountLEFT_FRONT = 0;
    return;
  } else {
    encoderCountRIGHT_FRONT = 0;
    return;
  }
}

void setDir(int i, int spd)
{
  if (spd < 0) {
    if (i == 0) {
      State_dirLEFT_FRONT = -1;
    }
    else if (i == 1) {
      State_dirLEFT_BACK = -1;
    }
    else if (i == 2) {
      State_dirRIGHT_FRONT = -1;
    }
    else if (i == 3) {
      State_dirRIGHT_BACK = -1;
    }
  }
  else
  {
    if (i == 0) {
      State_dirLEFT_FRONT = 1;
    }
    else if (i == 1) {
      State_dirLEFT_BACK = 1;
    }
    else if (i == 2) {
      State_dirRIGHT_FRONT = 1;
    }
    else if (i == 3) {
      State_dirRIGHT_BACK = 1;
    }
  }
}
#else
#error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif
