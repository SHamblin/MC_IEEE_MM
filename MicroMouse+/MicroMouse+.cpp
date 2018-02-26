//The libarys that this program requiers and the program it's self has been writen by Spencer Hamblin (spencerhamblin@gmail.com)

#include "MMLib.h"//Include for custom MM library
uint8_t IRDistance[5];//Global int for IR values

int main()
{
	//motorSpeed(50,50);
	//initalSetUp();
	DDRC &=  0b11110000;
	PORTC &= 0b11111101;
	PORTC |= 0b00001100;
	//BUTTON_1
	while(BUTTON_1 > 0){
		_delay_ms(50);
	}
	motorSpeedLeft(25);
	_delay_ms(250);
	motorSpeedLeft(0);
	//motorSpeedRight(25);
	delayS(1);
	//motorTicksPerSecond(4,4);
	backAlign();
	delayS(5);
	
	//motorTicks(5000,25);
	
	I2C_MODE_NORMAL
	PORTC = 0b00110000;
	
	delayS(2);
	
	//_delay_ms(5);
	

	
	I2CStart(I2C_MULTIPLEXER_WRITE);
	I2CDataSend(0b00000100);
	I2CStop();
	
	//_delay_ms(1);
	_delay_us(500);
	
	//PORTC5 = 0;
	DDRC |= 0b00100000;
	for(int i = 0; i< 20; i++){
		PORTC &= 0b11011111;
		_delay_us(7);
		PORTC |= 0b00100000;
		_delay_us(7);
		
	}
	DDRC &= 0b11011111;

	I2CStart(0xAA);
	I2CDataSend(0xAA);
	I2CStop();
	
	_delay_us(500);	
	_delay_ms(2);

	//Step 2 Write to 83h(LED current settings)
	I2CStart(IR_WRITE);
	I2CDataSend(0x83);
	I2CDataSend(20);//Sets LED current to 200 mA at 20d
	I2CStop();	
	
	//_delay_ms(1);	
	_delay_us(100);

	I2CStart(IR_WRITE);
	I2CDataSend(0x8F);
	I2CDataSend(3);//Sets it to 1.9 measurements/s
	I2CStop();	
	
	_delay_us(100);
	
	//Sets up periodic measurements
	I2CStart(IR_WRITE);
	I2CDataSend(0x89);
	I2CDataSend(0x8);//Enables auto measure
	I2CStop();
	
	//_delay_ms(1);
	_delay_us(100);

	

	

	
	//Step 1 Write to 82h(Proximity rate register)
	I2CStart(IR_WRITE);
	I2CDataSend(0x82);
	I2CDataSend(0b00000101);//Sets it to 62.5 measurements/s
	I2CStop();
	
	//_delay_ms(1);
	_delay_us(100);

	
	//_delay_ms(1);

	//I2CStart(IR_WRITE);
	//I2CDataSend(0x89);
	//I2CDataSend(2);//Enables auto measure
	//I2CStop();
	
	//_delay_ms(1);
		
	//I2CStart(IR_WRITE);
	//I2CDataSend(0x8A);
	//I2CDataSend(129);//Enables auto measure
	//I2CStop();
	
	//_delay_ms(1);	
	
	_delay_us(100);
	
	//_delay_ms(1);
	
	uint8_t command = 0;
	
	while(1){
		I2CStart(IR_WRITE);
		I2CDataSend(0x8E);
		I2CDataSend(0x0F);
		I2CStop();	
		
		_delay_us(10);
		
		I2CStart(IR_WRITE);
		I2CDataSend(0x80);
		I2CDataSend(0x08);
		I2CStop();		
		
		while(1){
			I2CStart(IR_WRITE);
			I2CDataSend(0x80);
			I2CStop();
			
			_delay_us(20);
			
			I2CStart(IR_READ);
			command = I2CDataRead(false);
			I2CStop();		
			
			_delay_us(20);
			
			if(command & 0x20){
				I2CStart(IR_WRITE);
				I2CDataSend(0x87);
				I2CStop();
				
				_delay_us(20);
				
				I2CStart(IR_READ);
				I2CDataRead(true);//OR's low byte into distance value
				I2CDataRead(false);
				I2CStop();		
				
				_delay_ms(1);
				
				
				I2CStart(IR_WRITE);
				I2CDataSend(0x8E);
				I2CDataSend(0x0F);
				I2CStop();
				
				_delay_us(10);
				
				I2CStart(IR_WRITE);
				I2CDataSend(0x80);
				I2CDataSend(0x08);
				I2CStop();		
				
			}
			
			delayS(1);
		}
	}
}