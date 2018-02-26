//This is header file for MicroMouseLib

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
//#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <math.h>

//Generic defines to make functions easer to read
#define I2C_MULTIPLEXER_WRITE 0b11101110
#define I2C_MULTIPLEXER_READ  0b11101111
#define IR_WRITE 0x26  
#define IR_READ  0x27  
#define I2C_MODE_NORMAL TWBR=32;
#define I2C_MODE_FAST   TWBR=2;
#define IR_LEFT        0b00010000
#define IR_FRONT_LEFT  0b00001000
#define IR_FRONT       0b00000100
#define IR_FRONT_RIGHT 0b00000010
#define IR_RIGHT       0b00000001
#define WHEEL_BASE 45 //Wheel base in MM
#define MOVE_DIS 18 //Distance to move 1 square
#define LEFT_MAX_SPEED 10//Trying to conpensate for differing motor speeds
#define RIGHT_MAX_SPEED 9

//Buttons == 0 when pressed
#define BUTTON_1 (PINC & 0b00000100)	//On PC2
#define BUTTON_3 (PINC & 0b00001000)	//On PC3
#define LEFT_ENCODER  (PINC & 0b00000001) //On PC0
#define RIGHT_ENCODER (PINC & 0b00000010) //On PC1

//int8_t I2CFail;

//Functions prototype
uint8_t I2CStart(uint8_t address);
uint8_t I2CDataSend(uint8_t address);
uint8_t I2CDataRead(bool more);
void I2CStop();
void IRsensorSelect(uint8_t number);
void initalSetUp();
void motorSpeedLeft(int8_t speed);
void motorSpeedRight(int8_t speed);
void motorSpeed(int8_t speedL, int8_t speedR);
void motorBrake(bool Left, bool Right);
void motorTicks(int leftTick,int8_t speed);
void motorTicksPerSecond(int leftTicksSec, int rightTicksSec);
void delay_mS(unsigned int mS);
void IRUpdate(uint8_t *IR);
void tone(unsigned int frequencey /*= 3000*/);
double lowBatt();
void delayS(unsigned int S);