//This is header file for MicroMouseLib

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
//#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <math.h>

#define INT16_MAX   0x7fff

//Generic defines to make functions easer to read
#define I2C_MULTIPLEXER_WRITE 0b11101110
#define I2C_MULTIPLEXER_READ  0b11101111
#define IR_WRITE 0x26  
#define IR_READ  0x27  
#define I2C_MODE_NORMAL TWBR=32;
#define I2C_MODE_FAST   TWBR=2;
#define IR_LEFT        0b00010000
#define IR_FRONT_LEFT  0b00001000//These are both old and un used after we decided we did not need diagnal sensors
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

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

//Structs
struct PID_STRUCT{
	int16_t P_Factor;//This stores the P(proportional) term of PID
	
	int16_t maxError;//This controls the max error, this lets us predict if a P*value will over flow the lint of int16_t	
};

//Functions prototype
uint8_t I2CStart(uint8_t address);
uint8_t I2CDataSend(uint8_t address);
uint8_t I2CDataRead(bool more);
void I2CStop();
void I2CPurge(uint8_t cycles);
void IRsensorSelect(uint8_t number);
void initalSetUp();
void setupIR();
void motorSpeedLeft(int8_t speed);
void motorSpeedRight(int8_t speed);
void motorSpeed(int8_t speedL, int8_t speedR);
void motorBrake(bool Left, bool Right);
void motorTicks(int leftTick,int8_t speed);
void motorTicksPerSecond(int leftTicksSec, int rightTicksSec);
void delay_mS(unsigned int mS);
void IRUpdate(uint8_t *IR);
unsigned int readIR(uint8_t sensor);
void tone(unsigned int frequencey /*= 3000*/);
double lowBatt();
void delayS(unsigned int S);
void backAlign();

void north(int heading);
void east(int heading);
void south(int heading);
void west(int heading);

void straight();
void leftTurn();
void rightTurn();
void turn180();

void moveMediumSpeed(unsigned int distanceMM);
void moveStraight(int16_t distance_CM, int16_t CM_Second_SetPoint);
void pidStructInit(const int16_t PTune, struct PID_STRUCT *pidStruct);
int16_t pidCalculation(int16_t setPoint, int16_t processValue, struct PID_STRUCT *pidStruct);

void motorSpeedLeft(uint8_t speed, bool reverse);
void motorSpeedRight(uint8_t speed, bool reverse);

void sensorTestCalibration();
void moveStraight();
void moveStraight2();

void beep();