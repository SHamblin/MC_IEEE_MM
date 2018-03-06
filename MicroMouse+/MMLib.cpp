//Micro Mouse libarry for MC 2/25/2015
//Created by Spencer Hamblin

#include "MMLib.h"//Header file with defines and function definitions

int8_t I2CStatus = 0,I2CWait = 0;

uint8_t I2CStart(uint8_t address)
{
	//PORTC = 0b00110000;//This sets the state of the ports. This makes sure it's ready to work
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

void I2CPurge(uint8_t cycles){//This purge command toggles the clock line 9 times for each cycle. This will clear junk left in the i2c buffer of i2c devices
	_delay_us(15);//This is simply a gap to make sure there is enough space between this and real I2C commands. This gap is for 400 Khz I2C
	TWCR = 0;//Zero TWCR register to so that if I2C was set up it can now be driven manualy
	DDRC |= 0b00100000;
	
	for(int i = 0; i< (9 * cycles); i++){
		PORTC |= 0b00100000;
		_delay_us(7);//This timeing works great for 400 KHz I2C
		PORTC &= 0b11011111;
		_delay_us(7);
	}
	PORTC = 0b00110000;//Re configure PORTC for I2C ussage just in case
	_delay_us(15);
}

void IRsensorSelect(uint8_t number)//Function to select the sensor on the I2C bus using the multiplexer
{
	I2CStart(I2C_MULTIPLEXER_WRITE);
	switch(number)
	{
		case IR_LEFT:		 I2CDataSend(IR_LEFT); break;
		//case IR_FRONT_LEFT:  I2CDataSend(IR_FRONT_LEFT); break;
		case IR_FRONT:		 I2CDataSend(IR_FRONT); break;
		case IR_FRONT_RIGHT: I2CDataSend(IR_FRONT_RIGHT); break;
		case IR_RIGHT:		 I2CDataSend(IR_RIGHT); break;
		default: break;
	}
	I2CStop();
}
//*****************************************************************************************
//******The above functions are only used internaly/by other functions*********************
//*****************************************************************************************

void initalSetUp()
{
	_delay_ms(10);//Gives time for every thing to stabalise, might not be needed
	DDRC &=  0b11110000;//Makes sure that the buttons and encoders are inputs
	PORTC = 0;//Makes sure all other parts of PORTC are initalised
	PORTC |= 0b00001111;//Enables pull up for the two buttons and helps assist the pull ups on the encoders
	
	//TWPS0=0; TWPS1=0;
	
	//ADC6 config code
	//ADMUX  = 0b01100110;//Some setings and slects ADC
	
	//ADCSRB = 0b00000000;
	
	//ADCSRA = 0b11000110;//Enables ADC and sets prescaler value
	
	
}

void setupIR(){//This function configures the IR sensors. It sets up all 3 connected sensors.
	
	I2CPurge(1);//This is to purge the sliter just in case
	
	for(int i = 0; i < 3; i++){
		switch(i)//This switch selects which sensor to set up
		{
			case 0: IRsensorSelect(IR_RIGHT);
					break;
			case 1: IRsensorSelect(IR_FRONT);
					break;			
			case 2: IRsensorSelect(IR_LEFT);
					break;
		}
		
		I2CPurge(2);
	
		I2CStart(IR_WRITE);//This sets the rate for automatic measurements
		I2CDataSend(0x82);
		I2CDataSend(0b00000101);//Sets it to 62.5 measurements/s
		I2CStop();	
	
		_delay_us(10);
	
		I2CStart(IR_WRITE);//This command sets the LED current from 0mA - 200mA
		I2CDataSend(0x83);
		I2CDataSend(20);//Sets LED current to 200 mA
		I2CStop();	
	
		_delay_us(10);
	
		I2CStart(IR_WRITE);//This sets the command register to enable automatic measurement. This should be the last setup command
		I2CDataSend(0x80);
		I2CDataSend(0b00000011);
		I2CStop();
	
		_delay_us(10);
	}
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

unsigned int readIR(uint8_t sensor){
	unsigned int result = 0;
	uint8_t resultMSB = 0;
	uint8_t resultLSB = 0;
	
	IRsensorSelect(sensor);
	
	I2CStart(IR_WRITE);//This selects the register to read from. In this case it's the most significant byte of the result
	I2CDataSend(0x87);
	I2CStop();
		
	_delay_us(10);
		
	I2CStart(IR_READ);
	resultMSB = I2CDataRead(true);//MSB
	resultLSB = I2CDataRead(false);//LSB
	I2CStop();	
	
	_delay_us(10);
	
	result = resultMSB;
	result = result << 8;//Shifts data from MSB to the right
	result += resultLSB;
	
	return result;
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

void moveMediumSpeed(unsigned int distanceMM){
	
}

void motorTicksPerSecond(int leftTicksSec, int rightTicksSec){
	const float PTune = 0.5;//This should be negative// P -0.5  I -0.001
	const float ITune = 0;//0.15;// -0.0001;  0.5
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
	delay_mS(loopTime+1000);
	totalTimeMs += loopTime;
	
	while(true){
		//totalTimeMs += loopTime;
		//Left PID Loop
		if(leftStatus != LEFT_ENCODER || (totalTimeMs - lastTickMeasuredL) > mSPerTickL){
			if(leftStatus != LEFT_ENCODER){
				MeasuredTicksL++;
			}
			
			//measuredSpeedL = (measuredSpeedL + ((totalTimeMs - lastTickMeasuredL)))/2;//This computes the speed. It's averaged over two results.
			measuredSpeedL = totalTimeMs - lastTickMeasuredL;
			
			PCompL = ((mSPerTickL - measuredSpeedL)) * PTune;
			//PCompL = ((measuredSpeedL - mSPerTickL)) * PTune;
			
			ICompL = ((totalTimeMs/mSPerTickL) - MeasuredTicksL) * ITune;
			
			//motorSpeedL = PCompL + ICompL;
			motorSpeedL = PCompL + ICompL;//biasL + 3;
			
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
			
			PCompR = ((mSPerTickR - measuredSpeedR)) * PTune;
			//PCompR = ((measuredSpeedR - mSPerTickR)) * PTune;
			
			ICompR = ((totalTimeMs/mSPerTickR) - MeasuredTicksR) * ITune;
			
			//motorSpeedR = PCompR + ICompR;
			motorSpeedR = PCompR + ICompR + 6;// + 20;// + biasR;
			
			if(biasR >= 100)biasR = 99;
			
			if(motorSpeedR <= 15)motorSpeedR = 16;
			
			
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

//This PID code is based on Atmel's app note on PID. To find the app note google AN_2558. http://www.microchip.com/wwwappnotes/appnotes.aspx?appnote=en591227
void moveStraight(int16_t distance_CM, int16_t CM_Second_SetPoint){//This is a PID algorithm for smooth and acurate motion control
	distance_CM *= 100;//Converts 2.26 CM to 226
	CM_Second_SetPoint *=100;
	//Wheel diameter was measured as 5.77 CM
	//#define WHEEL_CIRCUMFRENCE 18.1//Measure ticks per rotation
	//For the current hall effect sensors I count 4 high sections or 8 edges per-rotation
	//Distance per tick = 18.1/8 = 2.26 CM
	int16_t msPerTick = ((int16_t)226*100)/(CM_Second_SetPoint/10);//This is simplifyed to avoid floats. In proper form it's 1000mS/(CM_Second_SetPoint/CM_Per_Tick)
	const int16_t PTune = 2;
	//const float ITune = 0;
	const int32_t distanceTick = 226;//2.26 CM  Casting only uses the LSB's meaning that a 32Bit int needs to be less than the max for a 16bit int, this is 32bit to temp convert a value to a big enough range
	int16_t CM_Second_Left = 0, CM_Second_Right = 0;
	struct PID_STRUCT pidStructL;
	struct PID_STRUCT pidStructR;
	
	//int16_t CM_Second_Measured;
	//int16_t error;
	int16_t totalTimeMs = 0;
	int16_t lastMeasurementLeft = 0, lastMeasurementRight = 0;
	const int16_t loopTime = 5;
	uint8_t leftStatus = LEFT_ENCODER;
	uint8_t rightStatus = RIGHT_ENCODER;
	int16_t leftPIDResult, rightPIDResult;
	bool leftReverse = false, rightReverse = false;
	uint8_t leftMotorSpeed, rightMotorSpeed;
	//error = CM_Second_SetPoint - CM_Second_Measured;// Error = set point - process value
	#define blank 0
	bool leftDone = false, rightDone = false;
	
	pidStructInit(PTune, &pidStructL);
	pidStructInit(PTune, &pidStructR);
	
	motorSpeed(29,25);//This is to get it started
	_delay_ms(250);
	
	while(!leftDone || !rightDone){
		if(!leftDone && leftStatus != LEFT_ENCODER){
			//CM_Second_Left = ((totalTimeMs - lastMeasurementLeft)/1000) * distanceTick;
			//CM_Second_Left = (int16_t)(((totalTimeMs - lastMeasurementLeft) * ((int32_t)distanceTick))/1000);//45.2 -> 0.45 CM/S
			CM_Second_Left = ((int16_t)(((totalTimeMs - lastMeasurementLeft) * distanceTick)/1000) + CM_Second_Left)/2;//45.2 -> 0.45 CM/S
			
			leftPIDResult = pidCalculation(CM_Second_SetPoint , CM_Second_Left, &pidStructL); 
			
			if(leftPIDResult < 0){
				//leftReverse = true;
				leftPIDResult = 10 << 8;
			}else{
				leftReverse = false;
			}
			
			//leftMotorSpeed = (leftPIDResult >> 7) * 0xFF;
			
			motorSpeedLeft(leftMotorSpeed + blank + 3, leftReverse);
			
			lastMeasurementLeft = totalTimeMs;
			leftStatus = LEFT_ENCODER;
		}
		msPerTick = 300;
		if(rightStatus != RIGHT_ENCODER || (lastMeasurementRight + msPerTick) < totalTimeMs){
		//if(rightStatus != RIGHT_ENCODER){	
			if(rightStatus != RIGHT_ENCODER){
				//CM_Second_Right = ((totalTimeMs - lastMeasurementRight)/1000.0) * distanceTick;//One floating point part, almost eliminated it!
				CM_Second_Right = (1000.0/(totalTimeMs - lastMeasurementRight)) * distanceTick;//One floating point part, almost eliminated it!	
			}else{
				//CM_Second_Right = (((totalTimeMs + msPerTick)- lastMeasurementRight)/1000.0) * distanceTick;
				CM_Second_Right = (1000.0/(totalTimeMs - lastMeasurementRight)) * distanceTick;
			}
			//CM_Second_Right = ((int16_t)(((totalTimeMs - lastMeasurementRight) * distanceTick)/1000) + CM_Second_Right)/2;//45.2 -> 0.45 CM/S   Also casting only uses the LSB's meaning that a 32Bit int needs to be less than the max for a 16bit int
			//CM_Second_SetPoint = 1000;
			uint16_t CM_Second_SetPoint2 = 1000;
			uint8_t left = 0x03, right = 0xE8;
			
			I2CStart(0x02);
			I2CDataSend(totalTimeMs >> 8);
			I2CDataSend(totalTimeMs & 0xFF);
			I2CDataSend(lastMeasurementRight >> 8);
			I2CDataSend(lastMeasurementRight & 0xFF);			
			
			I2CDataSend(CM_Second_Right >> 8);
			I2CDataSend(CM_Second_Right & 0xFF);
			//I2CDataSend(CM_Second_SetPoint >> 8);
			//I2CDataSend(CM_Second_SetPoint & 0xFF);
			//I2CDataSend(left);
			//I2CDataSend(right);
			I2CStop();
			
			rightPIDResult = pidCalculation(CM_Second_SetPoint , CM_Second_Right, &pidStructR); 
			
			if(rightPIDResult < 0){
				//rightReverse = true;
				rightPIDResult = 10 << 8;
				}else{
				rightReverse = false;
			}
			
			rightMotorSpeed = (rightPIDResult >> 7) & 0xFF;
			
			motorSpeedRight(rightMotorSpeed + blank, rightReverse);
			
			I2CStart(0x02);
			I2CDataSend(rightPIDResult >> 8);
			I2CDataSend(rightPIDResult & 0xFF);
			I2CDataSend(rightMotorSpeed);
			I2CStop();
			
			//motorSpeedRight(pidCalculation(CM_Second_SetPoint , CM_Second_Right, pidStructR), false);
			
			if(rightStatus != RIGHT_ENCODER){
				lastMeasurementRight = totalTimeMs;
				rightStatus = RIGHT_ENCODER;
			}
		}
		_delay_ms(loopTime);
		totalTimeMs += loopTime;
	}
}

void pidStructInit(const int16_t PTune, struct PID_STRUCT *pidStruct){
	pidStruct->P_Factor = PTune; 
	pidStruct->maxError = INT16_MAX / (pidStruct->P_Factor + 1);
}

int16_t pidCalculation(int16_t setPoint, int16_t processValue, struct PID_STRUCT *pidStruct){
	int16_t p_term, error;
	int16_t outPut;
	
	error = setPoint - processValue;
	

	
	if(error > pidStruct->maxError){
		p_term = INT16_MAX;
	}else if(error < -pidStruct->maxError){
		p_term = -INT16_MAX;
	}else{
		//p_term = pidStruct->P_Factor * error;
		p_term = 3 * error;
	}
	
	I2CStart(0x04);
	I2CDataSend(error >> 8);
	I2CDataSend( error & 0xff);
	I2CDataSend(p_term >> 8);
	I2CDataSend( p_term & 0xff);	
	I2CStop();
		
	outPut = p_term;
	
	//if(outPut < 0)outPut 
	
	//return ((uint8_t)outPut);
	return outPut;
}

void motorSpeedLeft(uint8_t speed, bool reverse){//The alternate motor functions use the full range of 0-255 instead of 0-100. This allows more resolution however an extra input is needed to set reverse
	//IN1 - OC1B - PB2         IN2 - OC1A - PB1
	DDRB  |= 0b00000110;//Sets ports as outputs
	TCCR1B = 0b00000011;//sets pre-scaler at 64
	
	if(reverse)//To go in reverse only use IN2
	{
		TCCR1A = 0b10000001;//disables pin PB1
		OCR1A  = speed;
	}
	else if(speed == 0) TCCR1A = 0b00000001;
	else
	{
		TCCR1A = 0b00100001;//disables pin PB2
		OCR1B  = speed;
	}	
}

void motorSpeedRight(uint8_t speed, bool reverse){
	//IN1 - OC0A - PD6         IN2 - OC0B - PD5
	DDRD  |= 0b01100000;//Sets ports direction
	TCCR0B = 0b00000011;//Sets pre-scale at 64
	
	if(reverse)//To go in reverse only use IN2
	{
		TCCR0A = 0b00100001; //Disables IN1 pin(PD6)
		OCR0B = speed;//Timer counter register 0B converts percent to 8 bit
	}
	else if(speed == 0) TCCR0A = 0b00000001;//Disables both out pins, non braking
	else
	{
		TCCR0A = 0b10000001; //Disables IN2 pin(PD5)
		OCR0A = speed;//Timer counter register 0A converts percent to 8 bit
	}	
}

void sensorTestCalibration(){
	//The expected behavior is described bellow
	//If greater than too close speed up
	//If less than close but greater than normal normal speed
	//If less than normal but greater than far slow 
	//If less than far normal speed
	
	#define CLOSE_R 4000
	#define NORMAL_R 3400
	#define FAR_R 2500
	
	uint16_t distance = 0;
	
	for(;;){
		distance = readIR(IR_RIGHT);
		
		if(distance > CLOSE_R){
			motorSpeedRight(100);
		}else if(distance > NORMAL_R){
			motorSpeedRight(50);
		}else if(distance > FAR_R){
			motorSpeedRight(20);
		}else{
			motorSpeedRight(0);
		}
	}
}

void moveStraight(){
	//motorSpeed(25,25);
	#define speedL 25
	#define speedR 25
	motorSpeed(speedL,speedR);
	
	
	#define CLOSE_R 4000
	#define NORMAL_R 3200
	#define FAR_R 2500
	
	#define CLOSE_L CLOSE_R
	#define NORMAL_L NORMAL_R
	#define FAR_L FAR_R
	
	int16_t distanceL = 0;	
	int16_t distanceR = 0;
	int16_t distanceFront = 0;
	uint8_t frontDelay = 0;
	#define ADJUST 3
	
	//for(;;){
	//	_delay_ms(10);
	//}
	
	for(;;){
		distanceL = readIR(IR_LEFT);
		distanceR = readIR(IR_RIGHT);
		distanceFront = readIR(IR_FRONT);
		
		if(distanceFront > 3500){
			frontDelay++;
		}else{
			frontDelay = 0;
		}
		
		if(frontDelay > 5){
			motorBrake(true,true);
			return;
		}
		
		if(distanceL > CLOSE_L){
			motorSpeedLeft(speedL + ADJUST);
			}else if(distanceL > NORMAL_L){
			motorSpeedLeft(speedL);
			}else if(distanceL > FAR_L){
			motorSpeedLeft(speedL - ADJUST - 2);
			}else{
			motorSpeedLeft(speedL);
		}
		
		if(distanceR > CLOSE_R){
			motorSpeedRight(speedR + ADJUST);
			}else if(distanceR > NORMAL_R){
			motorSpeedRight(speedR);
			}else if(distanceR > FAR_R){
			motorSpeedRight(speedR - ADJUST - 2);
			}else{
			motorSpeedRight(speedR);
		}
		
		_delay_ms(10);		
	}
}

void moveStraight2(){
	//motorSpeed(25,25);
	#define speedL 63
	#define speedR 70
	//motorSpeed(speedL,speedR);
	motorSpeedLeft(speedL,false);
	motorSpeedRight(speedR,false);
	
	
	#define CLOSE_R 4000
	#define NORMAL_R 3200
	#define FAR_R 4000
	
	#define CLOSE_L CLOSE_R
	#define NORMAL_L NORMAL_R
	#define FAR_L FAR_R
	
	int16_t distanceL = 0;
	int16_t distanceR = 0;
	int16_t distanceFront = 0;
	float error = 0;
	float lastError = 0;
	float sumError = 0;
	int16_t p_term = 0;
	int16_t i_term = 0;
	int16_t dTerm = 0;
	uint8_t frontDelay = 0;
	#define ADJUST 3
	
	//for(;;){
	//	_delay_ms(10);
	//}
	
	for(;;){
		distanceL = readIR(IR_LEFT);
		distanceR = readIR(IR_RIGHT);
		distanceFront = readIR(IR_FRONT);
		//error = distanceR - distanceL;
		//error *= 0.05;
		
		
		//if(distanceR > FAR_R && distanceL > FAR_L){
		distanceL += 300;//2000;//7000;//14000
		error = distanceR - distanceL;
		p_term = error * 0.005;//0.01;//0.005;//0.003
			
			//error -= 5;
			
		//}else if(distanceR > FAR_R){//No wall left
			//error = distanceR - 3000;
			//error *= 0.05;
		//}else if(distanceL > FAR_L){//No wall right
			//error = NORMAL_R - 3000;
			//error *= 0.05;
		//}else{
			//break;
		//}
		
		#define P_MAX 20
		
		if(p_term > P_MAX){
			p_term = P_MAX;
		}else if(p_term < -P_MAX){
			p_term = -P_MAX;
		}
		
		dTerm = 0.0 * (lastError - error);
		
		if(error > 10){
			//error = 0;
		}else if(error < 10){
			//error = -10;
		}
		
		sumError = sumError + error;
		i_term = sumError * 0.1;
		
		if ( speedR + error < 0) error = -speedR;
		motorSpeedRight((speedR + p_term) + dTerm + i_term, false);
		if ( speedL - error < 0) error = speedL;
		motorSpeedLeft((speedL - p_term) - dTerm - i_term,false);
		
		lastError = error;
		
		if(distanceFront > 3500){
			frontDelay++;
		}else{
			frontDelay = 0;
		}
		
		if(frontDelay > 5){
			//motorBrake(true,true);
			//return;
		}
		
		
		
		_delay_ms(10);
	}
}

void beep(){
	DDRD  |= 0b00010000;
	PORTD |= 0b00010000;
	
	_delay_ms(100);
	
	PORTD &= 0b11101111;
}