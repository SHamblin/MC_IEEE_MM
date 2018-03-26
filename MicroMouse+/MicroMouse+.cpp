//The libarys that this program requiers and the program it's self has been writen by Spencer Hamblin (spencerhamblin@gmail.com)

#include "MMLib.h"//Include for custom MM library
uint8_t IRDistance[5];//Global int for IR values

int main()
{
	setUpInital();
	setUpADC();
	setUpIMU();
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
	
	beep();
	
	lowBatt();
	
	delayS(2);
	//backAlign();
	//delayS(10);
	//leftTurnGyro();
	//rightTurnGyro();
	//moveStraightGyro();
	moveStraightGyro(20000);//49    Circumference is 186.9mm  so 11.6mm per step   46 should be 3 squares  45 is ideal
	motorSpeedLeft(0);
	motorSpeedRight(0);
	_delay_ms(500);
	
	leftTurnGyro();
	//motorBrake(true,true);
	//motorSpeedLeft(255,true);
	//motorSpeedRight(255,false);
	//
	for(;;){
		_delay_ms(10);
	}
	//leftTurnGyro();
	
	_delay_ms(500);
	
	//rightTurnGyro();
	motorBrake(true,true);
	beep();
	
	for(;;){
		_delay_ms(10);
	}
	
	
	int16_t gyroZ;
	
	for(;;){
		gyroZ = readGyroZ();
		
		if(gyroZ < -200){
			beep();
		}
		_delay_ms(20);
	}
	
	delayS(2);
	backAlign();
	moveStraight(49);
	leftTurn();
	motorBrake(true,true);
	
	delayS(200);
	
	//int dis = 0;	
	//
	//for(;;){
		//dis = readIR(IR_RIGHT);
		//
		//if(dis > 3000){
			//motorSpeedLeft(25);//Too far
		//}else{
			//motorSpeedLeft(28);
		//}
	//}
	//
	//for(;;){//This is demo code to test the IR sensors
		//dis = readIR(IR_RIGHT);
		//
		//if(dis > 4000){
			//motorSpeedLeft(20);//Too close
		//}if(dis > 3000){
			//motorSpeedLeft(29);//Just Right
		//}if(dis > 2500){
			//motorSpeedLeft(35);//Too far
		//}else{
			////motorSpeedLeft(0);
		//}
		//
		//dis = readIR(IR_RIGHT);
		//if(dis > 3500){
			////motorSpeedRight(30);
			//}else{
			////motorSpeedRight(0);
		//}		
		//_delay_ms(16);
	//}
}