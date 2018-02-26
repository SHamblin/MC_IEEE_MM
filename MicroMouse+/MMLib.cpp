//Micro Mouse libarry for MC 2/25/2015
//Created by Spencer Hamblin

#include "MMLib.h"//Header file with defines and function definitions

int8_t I2CStatus = 0,I2CWait = 0;

uint8_t I2CStart(uint8_t address)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	return(TWSR);
}

uint8_t I2CDataSend(uint8_t data)
{
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	return(TWSR);
}

uint8_t I2CDataRead(bool more)
{
	int timeOut=0;
	if(more){
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);//starts bus read
		}else{
		TWCR = (1<<TWINT) | (1<<TWEN);//starts bus read
	}
	while(!(TWCR & (1<<TWINT)))
	{
		timeOut++;
		if(timeOut >= 100)break;
	}
	return(TWDR);
	//TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
}

void I2CStop()
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);//|(1<<TWIE);
	//(TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN)|(1<<TWIE))
}

void IRsensorSelect(uint8_t number)//Function to select the sensor on the I2C bus using the multiplexer
{
	I2CStart(I2C_MULTIPLEXER_WRITE);
	//I2CDataSend(0xFF);
	switch(number)
	{
		case 0: I2CDataSend(IR_LEFT); break;
		case 1: I2CDataSend(IR_FRONT_LEFT); break;
		case 2: I2CDataSend(IR_FRONT); break;
		case 3: I2CDataSend(IR_FRONT_RIGHT); break;
		case 4: I2CDataSend(IR_RIGHT); break;
		default: break;
	}
	I2CStop();
}
//*****************************************************************************************
//******The above functions are only used internaly/by other functions*********************
//*****************************************************************************************

void initalSetUp()
{
	//int8_t loopCount=0;
	_delay_ms(25);//Gives time for sensors to initialize
	DDRC =  0b00000000; //Sets PORTC data direction
	PORTC = 0b00001100; // activates internal pull-up 
	//TWPS0=0; TWPS1=0;
	
	//ADC6 config code
	ADMUX  = 0b01100110;//Some setings and slects ADC
	
	ADCSRB = 0b00000000;
	
	ADCSRA = 0b11000110;//Enables ADC and sets prescaler value
	
	////Set up registers for I2C
	//I2C_MODE_FAST; //Set I2C mode
	//for(int8_t loopCount=0;loopCount!=5; loopCount++)//Preforms initial config of IR sensors
	//{
		//IRsensorSelect(loopCount);
		////Step 1 Write to 82h(Proximity rate register)
		//I2CStart(IR_WRITE);
		//I2CDataSend(0x82);
		//I2CDataSend(0b101);//Sets it to 62.5 measurements/s
		//I2CStop();
		////Step 2 Write to 83h(LED current settings)
		//I2CStart(IR_WRITE);
		//I2CDataSend(0x83);
		//I2CDataSend(20);//Sets LED current to 200 mA at 20d
		//I2CStop();
		////Sets sensors 1 & 3 to different frequency's so not to interfere with 0,2,4
		///*if((loopCount=1)||(loopCount=3))
		//{
			//I2CStart(IR_WRITE);
			//I2CDataSend(0x8F);
			//I2CDataSend(0b00001001);//Sets it to 781.25 kHz
			//I2CStop();
		//}*/
		////Sets up periodic measurements
		//I2CStart(IR_WRITE);
		//I2CDataSend(0x80);
		//I2CDataSend(0b00000011);//Enables auto measure
		//I2CStop();
		////loopCount++;
		//_delay_ms(1);
	//}
	//sei();//Enables interrupts
}

void motorSpeedLeft(int8_t speed)
{
	
	//speed *= RIGHT_MAX_SPEED;//Limits motor speed
	
	//IN1 - OC1B - PB2         IN2 - OC1A - PB1
	DDRB  |= 0b00000110;//Sets ports as outputs
	TCCR1B = 0b00000011;//sets pre-scaler at 64
	if(speed < 0)//To go in reverse only use IN2
	{
		//speed &= 0b01111111;
		speed *= -1;
		//TCCR1A = 0b00100001;//disables puin PB1
		TCCR1A = 0b10000001;//disables puin PB1
		OCR1A  = speed*2.55;
	}
	else if(speed == 0) TCCR1A = 0b00000001;
	else
	{
		//TCCR1A = 0b10000000;
		//TCCR1A = 0b10000001;//disables puin PB2
		TCCR1A = 0b00100001;//disables puin PB2
		OCR1B  = speed*2.55;
	}	
	
}

void motorSpeedRight(int8_t speed)
{
	//speed *= LEFT_MAX_SPEED;//Limits motor speed
	
	//IN1 - OC0A - PD6         IN2 - OC0B - PD5
	DDRD  |= 0b01100000;//Sets ports direction
	//TCCR0A = 0b10100001;//Sets mode of operation, set to dual slope PWM mode
	TCCR0B = 0b00000011;//Sets pre-scale
	if(speed < 0)//To go in reverse only use IN2
	{
		//speed &= 0b01111111;//Removes negative component
		speed *= -1;
		//DDRD  &= 0b00100000;//Disables IN1 pin(PD6)
		TCCR0A = 0b00100001; //Disables IN1 pin(PD6)
		OCR0B = speed*2.55;//Timer counter register 0B converts percent to 8 bit
	}
	else if(speed == 0) TCCR0A = 0b00000001;//Disables both out pins, non braking
	else
	{
		//speed &= 0b01111111;//error checking
		TCCR0A = 0b10000001; //Disables IN2 pin(PD5)
		OCR0A = speed*2.55;//Timer counter register 0A converts percent to 8 bit
	}
}

void motorSpeed(int8_t speedL, int8_t speedR)//Speed is in percent 0-100
{
	motorSpeedLeft(speedL);
	motorSpeedRight(speedR);
}

void motorBrake(bool Left, bool Right)
{
	if(Left)
	{
		TCCR0A = 0b00000001;//Disables out put pins for timers and enables manual control
		PORTD =  0b01100000;//Puts both out high and enables braking
	}
	
	if(Right)
	{
		TCCR1A = 0b00000001;//Disables out put pins for timers and enables manual control
		PORTB  = 0b00000110;//Puts both out high and enables braking
	}
}

void motorTicks(int tickMax,int8_t speed)//Moves wheels for specified ticks
{
	DDRC &= 0b11111100;
	PORTC &= 0b11111100;
	//char status = (PINC && ENCODER_MASK);//Mask for PIND
	int8_t leftStatus = LEFT_ENCODER;
	int8_t rightStatus = RIGHT_ENCODER;
	int leftTick = 0;
	int rightTick = leftTick;
	int8_t speedLeft = speed;
	int8_t speedRight = speed * 0.5;
	//PC0 - Left        PC1 - Right
	//DDRC &= 0b11110011;//Makes sure DDRD is set right
	motorSpeed(speed,speed);
	while((leftTick < tickMax) && (rightTick < tickMax))
	{
		if(LEFT_ENCODER != leftStatus)//Left ticks see if pin has changed
		{
			leftStatus = LEFT_ENCODER;//Reused mask
			leftTick++;
		}
		
		if(RIGHT_ENCODER != rightStatus)//Right ticks see if pin has changed
		{
			rightStatus = RIGHT_ENCODER;//Reused mask
			rightTick++;
		}
		
		//if(leftTick < rightTick)motorSpeed(speed*.9,speed);
		//else if(leftTick > rightTick)motorSpeed(speed,speed*.9);
		
		if(leftTick < rightTick){
			if(speedRight > 17)speedRight -= 1;
			if(speedLeft < speed)speedLeft +=1;
			
		}else if(leftTick > rightTick ){
			if(speedLeft > 17)speedLeft -= 1;
			if(speedRight < speed)speedRight +=1;
		}
		
		motorSpeed(speedLeft,speedRight);
		//rightTick = 1;
		_delay_ms(20);
	}
	
	motorBrake(true, true);
}

void motorTicksPerSecond(int leftTicksSec, int rightTicksSec){
	float PTune = 0.3;//This should be negative// P -0.5  I -0.001
	float ITune = 0.15;// -0.0001;  0.5
	float PCompL = 0;
	float PCompR = 0;
	float ICompL = 0;
	float ICompR = 0 ;
	float MeasuredTicksL = 0;
	float MeasuredTicksR = 0;
	int loopTime = 5;
	int totalTimeMs = 0;
	int lastTickMeasuredL = 0;
	int lastTickMeasuredR = 0;
	int measuredSpeedL = 0;
	int measuredSpeedR = 0;
	int motorSpeedL;
	int motorSpeedR;
	
	float biasL = 25;//Free running this is ~ 5.5 ticks
	float biasR = 25;
	
	int mSPerTickL = 1000/leftTicksSec;
	int mSPerTickR = 1000/rightTicksSec;
	
	int8_t leftStatus = LEFT_ENCODER;
	int8_t rightStatus = RIGHT_ENCODER;

	motorSpeedLeft(biasL);
	motorSpeedRight(biasR);
	delay_mS(loopTime);
	totalTimeMs += loopTime;
	
	while(true){
		
		//Left PID Loop
		if(leftStatus != LEFT_ENCODER || (totalTimeMs - lastTickMeasuredL) > mSPerTickL){
			if(leftStatus != LEFT_ENCODER){
				MeasuredTicksL++;
			}
			
			//measuredSpeedL = (measuredSpeedL + ((totalTimeMs - lastTickMeasuredL)))/2;//This computes the speed. It's averaged over two results.
			measuredSpeedL = totalTimeMs - lastTickMeasuredL;
			
			PCompL = ((mSPerTickL - measuredSpeedL)) * PTune;
			
			ICompL = ((totalTimeMs/mSPerTickL) - MeasuredTicksL) * ITune;
			
			//motorSpeedL = PCompL + ICompL;
			motorSpeedL = PCompL + ICompL + biasL;
			
			if(biasL >= 100)biasL = 99;
			
			if(motorSpeedL <= 15)motorSpeedL = 16;
			
			//motorSpeedL = biasL;//Reviersices the motor as a fix for negative P
			
			motorSpeedLeft(motorSpeedL);
			
			
			
			if(leftStatus != LEFT_ENCODER){
				lastTickMeasuredL = totalTimeMs;
				leftStatus = LEFT_ENCODER;
			}
		}
		
		//else if((totalTimeMs - lastTickMeasuredL) > mSPerTickL){//Same loop but no update		}
		
		//Right PID Loop
		if(rightStatus != RIGHT_ENCODER || (totalTimeMs - lastTickMeasuredR) > mSPerTickR){
			if(rightStatus != RIGHT_ENCODER){
				MeasuredTicksR++;
			}
			
			//measuredSpeedR = (measuredSpeedR + ((totalTimeMs - lastTickMeasuredR)))/2;//This computes the speed. It's averaged over two results.
			measuredSpeedR = totalTimeMs - lastTickMeasuredR;
			
			PCompR = (mSPerTickR - measuredSpeedR) * PTune;
			
			ICompR = ((totalTimeMs/mSPerTickR) - MeasuredTicksR) * ITune;
			
			//motorSpeedR = PCompR + ICompR;
			motorSpeedR = PCompR + ICompR + biasR;
			
			if(biasR >= 100)biasR = 99;
			
			if(measuredSpeedR <= 15)measuredSpeedR = 16;
			
						
			//motorSpeedR = biasR;//Reviersices the motor as a fix for negative P
			
			motorSpeedRight(motorSpeedR);			
			
			if(rightStatus != RIGHT_ENCODER){
				lastTickMeasuredR = totalTimeMs;
				rightStatus = RIGHT_ENCODER;
			}
		}
		
		delay_mS(loopTime);
		totalTimeMs += loopTime;
	}
	
	
}

void delay_mS(unsigned int mS)//Wait time in miliseconds
{
	while(mS > 0)
	{
		if(mS >= 1000)
		{
			_delay_ms(1000);
			mS -= 1000;
		}else if(mS >= 100)
		{
			_delay_ms(100);
			mS -= 100;
		}else if(mS >= 10)
		{
			_delay_ms(10);
			mS -= 10;
		}else
		{
			_delay_ms(1);
			mS -= 1;
		}
	}
}

void IRUpdate(uint8_t *IR)
{
	uint8_t loopCount = 0;
	while( loopCount < 5 )
	{
		if(loopCount == (1 || 3) ){loopCount++; continue;}//Skips sensors Front Left, Front right
		IRsensorSelect(loopCount);
		I2CStart(IR_READ);
		I2CDataSend(0x87);
		IR[loopCount] = I2CDataRead(0);//Reads high byte
		I2CStop();
		IR[loopCount] = IR[loopCount] << 8;//Shits high byte into high part of 16 bit int
		
		I2CStart(IR_READ);
		I2CDataSend(0x88);
		IR[loopCount] |= I2CDataRead(0);//OR's low byte into distance value
		I2CStop();
		loopCount++;
	}
}

void tone(unsigned int frequencey = 3000)
{
	//OCR2A-PB3     OCR2B-PD3
	//OCR2A formula  (F_CPU/(frequency*2N)) -1
	DDRB |= 0b00001000;//sets pin PB3/15/OC2A to out put
	if(frequencey == 0)TCCR2A = 0b00000010;
	else
	{
		//maximum = F_CPU/(2*1024*(1+1));
		TCCR2A = 0b01000010;//Enables OCR2A and sets CTC mode
		if(frequencey < F_CPU/(2*1024*(1+1)) )
		{
			TCCR2B = 0b00000111;//Sets pre-scaler at 1024
			OCR2A = (8000000/(frequencey * 2048))-1;
		}
		else if(frequencey < F_CPU/(2*256*(1+1)) )
		{
			TCCR2B = 0b00000110;//Sets pre-scaler at 256
			OCR2A = (8000000/(frequencey * 512))-1;
		}
		else if(frequencey < F_CPU/(2*128*(1+1)) )
		{
			TCCR2B = 0b00000101;//Sets pre-scaler at 128
			OCR2A = (8000000/(frequencey * 256))-1;
		}
		else if(frequencey < F_CPU/(2*64*(1+1)) )
		{
			TCCR2B = 0b00000100;//Sets pre-scaler at 64
			OCR2A = (8000000/(frequencey * 128))-1;
		}
	}
}

double lowBatt()//Voltage divider is on ADC6
{
	//1 bit out of 256 is ~.0129 Volts  When Batt V = 9 vout = 3
	//When Batt V = 6.4  vout = 2.133 V
	//double BattV = ADCH
	if(ADCH <= 160)//only reads the upper byte
	{
		while(1)//Locks code into low batt alarm
		{
			tone();
			_delay_ms(500);
			tone(0);
			_delay_ms(500);;
		}
	}
	return((ADCH * 0.0129) * (1/3));//Returns battery voltage
}

void delayS(unsigned int S)//Wait time in seconds
{
	while(S > 0)
	{
		_delay_ms(1000);
		S--;
	}
}

void backAlign(){//Back Align for walls
	motorSpeed(-25,-20);
	delayS(2);
	motorSpeed(0,0);
}