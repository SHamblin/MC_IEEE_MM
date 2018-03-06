//The libarys that this program requiers and the program it's self has been writen by Spencer Hamblin (spencerhamblin@gmail.com)

#include "MMLib.h"//Include for custom MM library
uint8_t IRDistance[5];//Global int for IR values

int main()
{
	initalSetUp();
	setupIR();
	I2C_MODE_FAST
	//motorSpeed(100,100);
	//for(;;){_delay_ms(10);}
	int presses = 0;
	while( presses < 4 ){//This block keeps the code from running until a button is pressed
		_delay_ms(10);
		
		if(BUTTON_1 > 0){
			presses++;
		}else{
			presses = 0;
		}
	}
	
	while(BUTTON_1 > 0){ 
		_delay_ms(50); 
	}
	
	//while(1){
	//	_delay_ms(250);
	//} 
	//motorSpeedLeft(25);//With out a buzzer this confirms that the code is running
	//_delay_ms(250);
	//motorSpeedLeft(0);
	beep();
	//motorTicksPerSecond(6,6);
	delayS(4);
	backAlign();
	delayS(2);
	//moveStraight(200,10);
	//sensorTestCalibration();
	moveStraight();
	//motorSpeed(29,24);
	//delayS(20);
	
	//motorTicks(3000,3000);
	
	I2C_MODE_NORMAL
	
	delayS(200);
	
	int dis = 0;	
	
	for(;;){
		dis = readIR(IR_RIGHT);
		
		if(dis > 3000){
			motorSpeedLeft(25);//Too far
		}else{
			motorSpeedLeft(28);
		}
	}
	
	for(;;){//This is demo code to test the IR sensors
		dis = readIR(IR_RIGHT);
		
		if(dis > 4000){
			motorSpeedLeft(20);//Too close
		}if(dis > 3000){
			motorSpeedLeft(29);//Just Right
		}if(dis > 2500){
			motorSpeedLeft(35);//Too far
		}else{
			//motorSpeedLeft(0);
		}
		
		dis = readIR(IR_RIGHT);
		if(dis > 3500){
			//motorSpeedRight(30);
			}else{
			//motorSpeedRight(0);
		}		
		_delay_ms(16);
	}
}