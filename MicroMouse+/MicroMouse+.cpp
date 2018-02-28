//The libarys that this program requiers and the program it's self has been writen by Spencer Hamblin (spencerhamblin@gmail.com)

#include "MMLib.h"//Include for custom MM library
uint8_t IRDistance[5];//Global int for IR values

int main()
{
	initalSetUp();
	setupIR();
	
	while(BUTTON_1 > 0){//This block keeps the code from running until a button is pressed
		_delay_ms(10);
	}
	motorSpeedLeft(25);//With out a buzzer this confirms that the code is running
	_delay_ms(250);
	motorSpeedLeft(0);
	
	//motorTicksPerSecond(4,4);
	
	//motorTicks(5000,25);
	
	I2C_MODE_NORMAL
	
	delayS(2);
	
	int dis = 0;	
	
	for(;;){//This is demo code to test the IR sensors
		dis = readIR(IR_LEFT);
		if(dis > 3000){
			motorSpeedLeft(30);
		}else{
			motorSpeedLeft(0);
		}
		
		_delay_ms(16);
	}
}