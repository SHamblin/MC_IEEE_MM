//The libarys that this program requiers and the program it's self has been writen by Spencer Hamblin (spencerhamblin@gmail.com)

#include "MMLib.h"//Include for custom MM library
uint8_t direction = NORTH;
uint8_t walls = 0;
volatile uint8_t command;// = 0;
volatile bool ready = false;
volatile bool I2CDataReady = false;
volatile uint8_t timesPinged = 0;
#define BOMB 42

#define TWI_STX_ADR_ACK 0xA8  // Own SLA+R has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK 0x80 //Addressed with SLA+W, ack returned
#define TWI_STX_DATA_ACK 0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK          0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received

volatile bool left = false;
volatile bool fwrd = false;
volatile bool right = false;
volatile bool reverse = false;
volatile bool beep1 = false;
volatile bool beep2 = false;
volatile bool bombBeep = false;

ISR(TWI_vect){//Interupt routine for I2C slave mode

	if(TWSR == TWI_STX_ADR_ACK || TWSR == TWI_STX_DATA_ACK){
		TWDR = walls;
		TWCR = (1<<TWEN)|                              // Enable TWI-interface and release TWI pins
			(1<<TWIE)|(1<<TWINT)|                      // Keep interrupt enabled and clear the flag
			(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Answer on next address match
			(0<<TWWC);
		
	}else if(TWSR == TWI_STX_DATA_NACK){
		TWDR = walls;
		
		TWCR = (1<<TWEN)|                                 // Enable TWI-interface and release TWI pins
             (1<<TWIE)|(1<<TWINT)|                      // Keep interrupt enabled and clear the flag
             (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Answer on next address match
             (0<<TWWC);                                 //			
	}else if(TWSR == TWI_SRX_ADR_DATA_ACK){
			
		command = TWDR;
			
		TWCR = (1<<TWEN)|                                 // TWI Interface enabled
			(1<<TWIE)|(1<<TWINT)|                      // Enable TWI Interupt and clear the flag to send byte
			(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|           // Send ACK after next reception
			(0<<TWWC);
			
		ready = false;//Command received, not ready untill command finished
		if(command == 1){
			left = true;
		}else if(command == 2){
			fwrd = true;
		}else if(command == 3){
			right = true;
		}else if(command == 4){
			reverse = true;	
		}else if(command == 9){
			beep1 = true;
		}else if(command == 10){
			beep2 = true;
		}else if(command == 42){
			bombBeep = true;
		}
	}
}

int presses = 0;
uint16_t irRead = 0;

uint8_t status;

int main()
{
	setUpInital();
	setUpADC();
	setUpIMU();
	setupIR();
	I2C_MODE_FAST
	sei();//Enable interupts, this is for I2C slave mode

	ready = false;

	for(;;){
		walls = readWalls2(direction);
		ready = true;
		
		while(ready){
			_delay_ms(10);
		}
	
		if(left){
			_delay_ms(200);
			leftTurnGyro();
			_delay_ms(500);
			moveStraightGyro(14);
			
			direction --;
			if(direction > 3 ) direction = 3;
			
		}else if(fwrd){
			_delay_ms(200);
			moveStraightGyro(14);
		}else if(right){
			_delay_ms(200);
			rightTurnGyro();
			_delay_ms(500);
			moveStraightGyro(14);
			
			direction++;
			
			if(direction > 3)direction = 0;
		}else if(reverse){
			_delay_ms(200);
			turn180();
			
			direction += 2;
			
			if(direction > 3)direction = direction - 4;
			 
		}else if(beep1){
			beep();
		}else if(beep2){
			beep();
			_delay_ms(200);
			beep();
		}else if(bombBeep){
			beep();
			_delay_ms(500);
			beep();
			_delay_ms(400);
			beep();
			_delay_ms(350);
			beep();
			_delay_ms(300);
			beep();
			_delay_ms(250);
			beep();
			_delay_ms(200);
			beep();
			_delay_ms(150);
			beep();
			_delay_ms(100);
			beep();
			_delay_ms(50);
			beep();
			_delay_ms(50);
			beep();
			_delay_ms(50);
			
			beep();
			beep();
			beep();
			beep();
			beep();
			beep();
			beep();
		}
		
		ready = true;
		
		left = false;
		fwrd = false;
		right = false;
		reverse = false;
		beep1 = false;
		beep2 = false;
		bombBeep = false;
	}	
}
	//for(;;){
		////beep();
		//if(!ready){
			//ready = true;
			//beep();
		//}
		//_delay_ms(2000);
	//}
	//
	//while( presses < 4 ){//This block keeps the code from running until a button is pressed
		//_delay_ms(10);
		//
		//if(BUTTON_1 > 0){
			//presses++;
		//}else{
			//presses = 0;
		//}
	//}
	//
	//while(BUTTON_1 > 0){ 
		//_delay_ms(50); 
	//}
	
	//for(;;){_delay_ms(10);}
			
	//beep();
	
	//lowBatt();
	//_delay_ms(2000);
	//leftTurnGyro();
	//_delay_ms(2000);
	
	//direction = SOUTH;
	//
	//south(&direction);
	//south(&direction);
	//south(&direction);
	//south(&direction);
	//south(&direction);
	//east(&direction);					
	//
	//north(&direction);
	//north(&direction);
	//north(&direction);
	//north(&direction);
	//
	//east(&direction);
	//east(&direction);
	//
	//south(&direction);
	//south(&direction);
	//
	//east(&direction);
	//south(&direction);
	//west(&direction);
	//south(&direction);
	//east(&direction);
	//east(&direction);	