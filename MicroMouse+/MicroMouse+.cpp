//The libarys that this program requiers and the program it's self has been writen by Spencer Hamblin (spencerhamblin@gmail.com)

#include "MMLib.h"//Include for custom MM library
uint8_t direction = NORTH;
uint8_t walls = 0;
volatile uint8_t command = 0;
volatile bool ready = false;

ISR(TWI_vect){//Interupt routine for I2C slave mode
	
}

int main()
{
	setUpInital();
	setUpADC();
	setUpIMU();
	setupIR();
	I2C_MODE_FAST
	sei();//Enable interupts, this is for I2C slave mode
	//beep();
	//motorSpeedBoth(60,60);
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
	_delay_ms(2000);
	//leftTurnGyro();
	_delay_ms(2000);
	uint16_t irRead = 0;
	//for(;;){
		////readMagHeading();
		////moveStraightGyro(1);
		////motorSpeedBoth(0,0);
		//irRead = readIR(IR_FRONT);
		//if(irRead > 4400) beep();
		//_delay_ms(50);
	//}

	//rightTurnGyro();
	//beep();
	//moveStraightGyro(75 + 4);
	moveStraightGyro(14);
	_delay_ms(500);
	moveStraightGyro(14);
	_delay_ms(500);
	moveStraightGyro(14);
	_delay_ms(500);
	moveStraightGyro(14);
	_delay_ms(500);
	rightTurnGyro();
	_delay_ms(500);
	moveStraightGyro(13);
	_delay_ms(500);
	moveStraightGyro(14);
	_delay_ms(500);
	rightTurnGyro();
	_delay_ms(500);
	moveStraightGyro(13);
	_delay_ms(500);
	moveStraightGyro(14);

	leftTurnGyro();
	_delay_ms(500);
	_delay_ms(500);
	moveStraightGyro(14);
	_delay_ms(500);
	moveStraightGyro(14);

	_delay_ms(500);
	rightTurnGyro();
	_delay_ms(500);
	moveStraightGyro(14);

	motorSpeedBoth(0,0);

	

	for(;;){
		//readMagHeading();
		//moveStraightGyro(1);
		//motorSpeedBoth(0,0);
		irRead = readIR(IR_RIGHT);
		if(irRead > 3300) beep();
		_delay_ms(50);
	}
	
	backAlign();
	delayS(23);
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
	
	//Main loop for the maze
	backAlign();
	for(;;){
		walls = readWalls(direction);
		
		while(!ready){
			_delay_ms(10);
		}
		
		switch(command){
			case NORTH:
				north(direction);
				direction = NORTH;
			break;
			case EAST:
				east(direction);
				direction = EAST;
			break;
			case SOUTH:
				south(direction);
				direction = SOUTH;
			break;
			case WEST:
				west(direction);
				direction = WEST;
			break;
		}
	}
	
}