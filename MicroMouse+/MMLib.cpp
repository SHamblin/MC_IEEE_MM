//Micro Mouse libarry for MC 2/25/2015
//Created by Spencer Hamblin

#include "MMLib.h"//Header file with defines and function definitions

int8_t I2CStatus = 0,I2CWait = 0;

uint8_t I2CStart(uint8_t address){//Transmits I2C start condition with the address
	TWCR = 0;//Clears TWCR first
	TWCR = (1<<TWINT)|(0<<TWEA)|(1<<TWSTA)|(1<<TWEN);//This enables and sets up an I2C start condition
	while(!(TWCR & (1<<TWINT)));//Waits for ready state?
	TWDR = address;//Passes address to transmit to the I2C hardware
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));//While transmit is not done
	return(TWSR);//Return status code, could check for error if you wanted
}

uint8_t I2CDataSend(uint8_t data){//Sends given data on the I2C bus
	TWDR = data;//Sends data to transmit to the I2C hardware
	TWCR = (1<<TWINT) | (1<<TWEN);//Starts data transmit at hardware level
	while(!(TWCR & (1<<TWINT)));//While data is sending
	return(TWSR);//Return status code, could check for errors if you wanted
}

uint8_t I2CDataRead(bool more){//Reads data off I2C bus. Only set more to true if there is a read right after
	if(more){//If more data to transmit after this data is sent
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);//starts bus read and will acknowledge
	}else{
		TWCR = (1<<TWINT) | (1<<TWEN);//starts bus read, will not acknowledge
	}
	while(!(TWCR & (1<<TWINT)));
	return(TWDR);
	//TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
}

void I2CStop(){//Stops and finishes I2C transmission
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);//Sets hardware for an I2C stop condition
	
	_delay_us(4);
	
	TWCR = 0;//Clear TWCR tp set up I2C Slave
	TWCR = (1<<TWEA)|(1<<TWEN);//Enable slave mode again
	
}

void I2CPurge(uint8_t cycles){//This purge command toggles the clock line 9 times for each cycle. This will clear junk left in the i2c buffer of i2c devices
	_delay_us(15);//This is simply a gap to make sure there is enough space between this and real I2C commands. This gap is for 400 Khz I2C
	TWCR = 0;//Zero TWCR register to so that if I2C was set up it can now be driven manualy
	DDRC |= 0b00100000;
	
	for(int i = 0; i< (9 * cycles); i++){
		PORTC |= 0b00100000;//Sink current to drive line low
		
		if(TWBR == 32){//32 is I2C_MODE_NORMAL
			_delay_us(28);//This timing works great for 100 KHz I2C
		}else if(TWBR == 2){//2 is I2C_MODE_FAST
			_delay_us(7);//This timing works great for 400 KHz I2C
		}else{
			_delay_us(28);//I can't see why this would be hit but it's here just in case
		}

		PORTC &= 0b11011111;//Sets high by stop sinking current
		
		if(TWBR == 32){
			_delay_us(28);//This timing works great for 100 KHz I2C
		}else if(TWBR == 2){
			_delay_us(7);//This timing works great for 400 KHz I2C
		}else{
			_delay_us(28);//I can't see why this would be hit but it's here just in case
		}
	}
	
	PORTC = 0b00110000;//Re configure PORTC for I2C usage just in case
	_delay_us(40);//Wait to make sure the bus stabalises, probably not needed here just in case
}

void I2CDebugOut(int8_t data){//I2C Debug outputs. Spits data out on the I2C to be grabed by the scope
	I2CStart(0x02);//Starts a write to address 1, LSB is used as R/W flag
	I2CDataSend(data);
	I2CStop();
}

void I2CDebugOut(uint8_t data){
	I2CStart(0x02);//Starts a write to address 1, LSB is used as R/W flag
	I2CDataSend(data);
	I2CStop();
}

void I2CDebugOut(int16_t data){
	I2CStart(0x02);//Starts a write to address 1, LSB is used as R/W flag
	I2CDataSend(data >> 8);//Sends MSByte
	I2CDataSend(data & 0xff);//Sends LSByte
	I2CStop();	
}

void I2CDebugOut(uint16_t data){
	I2CStart(0x02);//Starts a write to address 1, LSB is used as R/W flag
	I2CDataSend(data >> 8);
	I2CDataSend(data & 0xff);
	I2CStop();	
}

void setUpInital(){//Initial basic set up of IO registers
	_delay_ms(10);//Gives time for every thing to stabalise, might not be needed
	
	I2C_MODE_FAST
	DDRC &=  0b11110000;//Makes sure that the buttons and encoders are inputs
	PORTC = 0;//Makes sure all other parts of PORTC are initalised
	PORTC |= 0b00001111;//Enables pull up for the two buttons and helps assist the pull ups on the encoders
	
	TWAR = 42;//Sets address when acting as a slave
}

void setUpADC(){;//Sets up ADC for use by battery alarm
	PRR &=   0b11111110;//turns on the adc by disabling adc power reduction
	ADMUX =  0b01100110; //First 2 bits enable external voltage reference, next left adjusts result, last for select ADC6
	ADCSRA = 0b10000110;//Enables ADC and sets pre-scaler of 64.
}

void setUpIMU(){//Sets up 9DOF IMU
	
	I2CPurge(1);
	
	I2CStart(IMU_GYRO_WRITE);
	I2CDataSend(0x10);//Select CTRL_REG1_G
	I2CDataSend(0b10010011);//Sets a Hz of 238 with cutoff of 78, and scale of 500 dps
	I2CStop();
}

void IRsensorSelect(uint8_t number){//Function to select the sensor on the I2C bus using the I2C multiplexer
	I2CStart(I2C_MULTIPLEXER_WRITE);//Starts communication with the multiplexer
	
	switch(number)
	{
		case IR_LEFT:		 I2CDataSend(IR_LEFT);
		break;
		case IR_FRONT_LEFT:  I2CDataSend(IR_FRONT_LEFT);
		break;
		case IR_FRONT:		 I2CDataSend(IR_FRONT);
		break;
		case IR_FRONT_RIGHT: I2CDataSend(IR_FRONT_RIGHT);
		break;
		case IR_RIGHT:		 I2CDataSend(IR_RIGHT);
		break;
		default: break;
	}
	
	I2CStop();
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
		
		I2CPurge(2);//Purges I2C clock line just in case
	
		I2CStart(IR_WRITE);//This sets the rate for automatic measurements
		I2CDataSend(0x82);
		//I2CDataSend(0b00000101);//Sets it to 62.5 measurements/s
		I2CDataSend(0b00000111);//Sets it to 250 measurements/s, this was in an effort to improve response time
		I2CStop();	
	
		_delay_us(10);//Delay between comands. Not sure if needed but I like the idea
	
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

uint16_t readIR(uint8_t sensor){//Reads from a single selected IR sensor
	uint16_t result = 0;
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
	result += resultLSB;//Adds in the LSB
	
	return result;
}

int16_t readGyroZ(){//Reads the Z axis of the gyro
	//Left is positive
	uint8_t MSByte, LSByte;
	int16_t result = 0;
	
	I2CStart(IMU_GYRO_WRITE);
	I2CDataSend(0x1C);//Selects OUT_Z_G MSB byte, value is expresesed in a 16bit word two's complement
	I2CStart(IMU_GYRO_READ);
	LSByte = I2CDataRead(true);
	MSByte = I2CDataRead(false);
	I2CStop();
	
	result = MSByte;
	result = result << 8;
	result += LSByte;
	
	return result;
}

void beep(){//Simple short beep
	DDRD  |= 0b00010000;
	PORTD |= 0b00010000;
	
	_delay_ms(100);
	
	PORTD &= 0b11101111;
}

void lowBatt(){//low battery warning. This is needed since a lipo is damaged if it gets bellow 3V per cell
	ADCSRA |= 0b01000000;
	
	//while(!(ADCSRA && 0b1000000)){//In theory I could keep checking this bit to know when the adc is done
	//	_delay_us(1);
	//}
	
	_delay_ms(100);
	
	if(ADCH < 185){//This works out to be about 7.2 volts, 217 is 8.4 volts
		for(;;){
			beep();
			_delay_ms(500);
		}
	}
}

void delayS(unsigned int S){//Wait time in seconds
	while(S > 0)
	{
		_delay_ms(1000);
		S--;
	}
}

void sensorTestCalibration(){//Diagnostic function to help test sensors at the competition, add more as needed
	//The expected behavior is described bellow
	//If greater than too close speed up
	//If less than close but greater than normal normal speed
	//If less than normal but greater than far slow
	//If less than far normal speed
	
	const uint16_t closeR = 4000;
	const uint16_t normalR = 3400;
	const uint16_t farR = 2500;
	
	uint16_t distance = 0;
	
	for(;;){
		distance = readIR(IR_RIGHT);
		
		if(distance > closeR){
			motorSpeedRight(100);
			}else if(distance > normalR){
			motorSpeedRight(50);
			}else if(distance > farR){
			motorSpeedRight(20);
			}else{
			motorSpeedRight(0);
		}
	}
}

void pidStructInit(const float PTune, const float ITune, struct PID_STRUCT *pidStruct){
	pidStruct->P_Factor = PTune;
	pidStruct->I_Factor = ITune;
	
	pidStruct->sumError = 0;
	pidStruct->maxError = INT8_MAX / (pidStruct->P_Factor + 1);
	pidStruct->maxSumError = (INT32_MAX / 2) / (pidStruct->I_Factor + 1);
}

int16_t pidCalculation(int16_t setPoint, int16_t processValue, struct PID_STRUCT *pidStruct){
	int16_t p_term, error;
	int32_t sumE, i_term;
	int32_t outPut;

	error = setPoint - processValue;
	
	//Calculate P Term
	if(error > pidStruct->maxError){
		p_term = INT8_MAX;
	}else if(error < -pidStruct->maxError){
		p_term = -INT8_MAX;
	}else{
		p_term = pidStruct->P_Factor * error;
	}

	p_term = pidStruct->P_Factor * error;
	
	//Calculate I Term
	sumE = pidStruct->sumError + error;
	//if(sumE > pidStruct->maxSumError){
		//i_term = (INT32_MAX/2);
		//pidStruct->sumError = pidStruct->maxSumError;
	//}else if(sumE < -pidStruct->maxSumError){
		//i_term = -(INT32_MAX/2);
		//pidStruct->sumError = -pidStruct->maxSumError;
	//}else{
		//pidStruct->sumError = sumE;
		//i_term = pidStruct->I_Factor * pidStruct->sumError;
	//}
	
	pidStruct->sumError = sumE;
	i_term = pidStruct->I_Factor * pidStruct->sumError;
	
	if(i_term > INT16_MAX){
		i_term = INT16_MAX - 1;
	}else if(i_term < -INT16_MAX){
		i_term = -INT16_MAX + 1;
	}
	
	//I2CStart(0x04);
	//I2CDataSend(error >> 8);
	//I2CDataSend( error & 0xff);
	//I2CDataSend(p_term >> 8);
	//I2CDataSend( p_term & 0xff);
	//I2CStop();

	outPut = p_term + i_term;
	//outPut = p_term;
	if(outPut > INT8_MAX){
		outPut = INT8_MAX;
	}else if(outPut < -INT8_MAX){
		outPut = -INT8_MAX;
	}
	//if(outPut < 0)outPut

	//return ((uint8_t)outPut);
	return ((int16_t)outPut);
}

void motorSpeedLeft(int8_t speed){//Old function, -100<->0<->100, limited resolution since it's range is 0-100 int

	//IN1 - OC1B - PB2         IN2 - OC1A - PB1
	DDRB  |= 0b00000110;//Sets ports as outputs
	TCCR1B = 0b00000011;//sets pre-scaler at 64
	if(speed < 0)//To go in reverse only use IN2
	{
		speed *= -1;
		TCCR1A = 0b10000001;//disables puin PB1
		OCR1A  = speed*2.55;
	}
	else if(speed == 0) TCCR1A = 0b00000001;
	else
	{
		TCCR1A = 0b00100001;//disables pin PB2
		OCR1B  = speed*2.55;
	}	
	
}

void motorSpeedRight(int8_t speed){//Old function, -100<->0<->100, limited resolution since it's range is 0-100 int

	//IN1 - OC0A - PD6         IN2 - OC0B - PD5
	DDRD  |= 0b01100000;//Sets ports direction
	TCCR0B = 0b00000011;//Sets pre-scale
	if(speed < 0)//To go in reverse only use IN2
	{
		speed *= -1;
		TCCR0A = 0b00100001; //Disables IN1 pin(PD6)
		OCR0B = speed*2.55;//Timer counter register 0B converts percent to 8 bit
	}
	else if(speed == 0) TCCR0A = 0b00000001;//Disables both out pins, non braking
	else
	{
		TCCR0A = 0b10000001; //Disables IN2 pin(PD5)
		OCR0A = speed*2.55;//Timer counter register 0A converts percent to 8 bit
	}
}

void motorSpeedBoth(int8_t speedL, int8_t speedR)//Old function to set both motors on a range of -100<->0<->100, Left  Right
{
	motorSpeedLeft(speedL);
	motorSpeedRight(speedR);
}

void motorSpeedLeft(uint8_t speed, bool reverse){//The motor function use the full range of 0-255 instead of 0-100. This allows more resolution however an extra input is needed to set reverse
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

void motorBrake(bool Left, bool Right)//Function to brake each motor
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





void backAlign(){//Back Align for walls
	motorSpeedBoth(20,20);
	//delayS(2);
	_delay_ms(500);
	//motorSpeed(0,0);
	
	//uint8_t leftEncode = LEFT_ENCODER;
	//uint8_t rightEncode = RIGHT_ENCODER;
	
	//for(;;){
		//if(leftEncode != LEFT_ENCODER){
			//motorSpeedLeft(0,false);
		//}
		//
		//if(rightEncode != RIGHT_ENCODER){
			//motorSpeedRight(0,false);
		//}
		//
		//if(leftEncode != LEFT_ENCODER && rightEncode != RIGHT_ENCODER){
			//break;
		//}
	//}
	motorSpeedBoth(0,0);
	//beep();
	_delay_ms(200);
	moveStraightGyro(8);
	beep();
	delayS(1);
	
}

void moveStraight(int16_t ticks){//This function will move the robot forward in ticks or CM, not sure which yet, IR distance sensors are used for wall avoidance
	ticks = ticks/2.15;

	#define speedL 70
	#define speedR 67
	//motorSpeed(speedL,speedR);
	motorSpeedLeft(speedL,false);
	motorSpeedRight(speedR,false);
		
	#define CLOSE_R 4000
	#define NORMAL_R 3200
	#define FAR_R 2500
	
	#define CLOSE_L CLOSE_R
	#define NORMAL_L NORMAL_R
	#define FAR_L FAR_R
	
	uint8_t leftStatus = LEFT_ENCODER;
	int16_t totalTicks = 0;
	
	int16_t distanceL = 0;
	int16_t distanceR = 0;
	int16_t error = 0;
	int16_t distanceFront = 0;
	uint8_t frontDelay = 0;
	#define ADJUST 5
		
	for(;;){
		distanceL = readIR(IR_LEFT);
		distanceR = readIR(IR_RIGHT);
		distanceFront = readIR(IR_FRONT);
		
		if(leftStatus != LEFT_ENCODER){
			totalTicks++;
			if(totalTicks == ticks){
				motorSpeedBoth(0,0);
				return;
			}
			leftStatus = LEFT_ENCODER;
		}
		
		error = distanceL - distanceR;
		
		if(distanceFront > 3500){
			frontDelay++;
			}else{
			frontDelay = 0;
		}
		
		if(frontDelay > 5){
			motorBrake(true,true);
			return;
		}
		
		if(error < -700){//Left wall is gone
			//if(true){
			if(distanceR > 3500){//too close
				motorSpeedLeft(speedL - ADJUST - 3, false);
				motorSpeedRight(speedR, false);
				}else{
				motorSpeedLeft(speedL, false);
				motorSpeedRight(speedR - ADJUST + 1, false);
			}
			//motorBrake(true,true);
			beep();
			
			}else if(error > 200){//Difting left
			motorSpeedLeft(speedL, false);
			motorSpeedRight(speedR - ADJUST, false);
			}else if(error < -200){//drifting right
			motorSpeedLeft(speedL - ADJUST - 5, false);
			motorSpeedRight(speedR, false);
			}else{//In range
			motorSpeedLeft(speedL, false);
			motorSpeedRight(speedR, false);
		}

		_delay_ms(10);
	}
}

void moveStraightGyro(){//Move forward using the gyro to keep the bot straight
	int16_t gyroValue = 0;
	const int16_t gyroOffset = readGyroZ();
	const float PTune = 0.0075;
	const float ITune = 0.001;
	struct PID_STRUCT pidStruct;
	int16_t pidReturn;
	
	const int8_t motorSpeed = 65;
	
	pidStructInit(PTune, ITune, &pidStruct);
	
	for(;;){
		gyroValue = readGyroZ() - gyroOffset;
		
		
		pidReturn = pidCalculation(0, gyroValue, &pidStruct);//Will return a negative value if drifing left
		
		I2CDebugOut(gyroValue);
		I2CDebugOut(pidReturn * -1);
		
		
		if(pidReturn + motorSpeed > motorSpeed * 2){
			pidReturn = motorSpeed;
		}else if(pidReturn + motorSpeed < 0){
			pidReturn = -motorSpeed;
		}
		
		motorSpeedLeft(motorSpeed - pidReturn,false);
		motorSpeedRight(motorSpeed + pidReturn,false);
		
		_delay_ms(10);
	}
}

void moveStraightGyro(uint16_t ticks){//Move forward using the gyro to keep the bot straight and travel a specified ticks
	int16_t gyroValue = 0;
	int16_t errorIR = 0;
	const int16_t gyroOffset = readGyroZ();
	const float PTune = 0.0075;
	const float ITune = 0.001;
	const float wallPTune = 0.0025;
	const float wallITune = 0.0007;
	struct PID_STRUCT pidStruct;
	struct PID_STRUCT irPidStruct;
	int16_t pidGyroReturn;
	int16_t pidIrReturn;
	int8_t leftStatus = LEFT_ENCODER;
	int8_t rightStatus = RIGHT_ENCODER;
	const int8_t motorSpeed = 65;
	uint16_t ticksRecoreded = 0;
	const int16_t maxErrorIR = 1000;
	
	pidStructInit(PTune, ITune, &pidStruct);
	pidStructInit(wallPTune, wallITune, &irPidStruct);
	
	for(;;){
		gyroValue = readGyroZ() - gyroOffset;
		errorIR = readIR(IR_LEFT) - readIR(IR_RIGHT);//Positive value if it deviates left same as the gyro
		

		
		pidGyroReturn = pidCalculation(0, gyroValue, &pidStruct);//Will return a negative value if drifing left
		pidIrReturn = pidCalculation(0, errorIR, &irPidStruct);

		if(errorIR > maxErrorIR || errorIR < -maxErrorIR){
			beep();
			errorIR = 0;
			pidIrReturn = 0;
			irPidStruct.sumError = 0;
		}
		
		I2CDebugOut(gyroValue);
		I2CDebugOut(pidGyroReturn * -1);
		
		
		if(pidGyroReturn + motorSpeed > motorSpeed * 2){
			pidGyroReturn = motorSpeed;
		}else if(pidGyroReturn + motorSpeed < 0){
			pidGyroReturn = -motorSpeed;
		}
		
		motorSpeedLeft(motorSpeed - pidGyroReturn - pidIrReturn,false);
		motorSpeedRight(motorSpeed + pidGyroReturn + pidIrReturn,false);
		
		

		if(LEFT_ENCODER != leftStatus){//Left ticks see if pin has changed
			leftStatus = LEFT_ENCODER;//Reused mask
			ticksRecoreded++;
		}

		if(RIGHT_ENCODER != rightStatus){//Right ticks see if pin has changed
			rightStatus = RIGHT_ENCODER;//Reused mask
			ticksRecoreded++;
		}		
		
		if(ticksRecoreded >= ticks){
			return;
		}
		
		_delay_ms(10);
	}
}

void leftTurnGyro(){//Preform a left turn using the gyro
	int16_t gyroValue = 0;
	const int16_t gyroOffset = readGyroZ();
	const float PTune = 0.007;
	const float ITune = 0.00003;
	struct PID_STRUCT pidStruct;
	int16_t pidReturn;
	const int16_t turnComplete = 2300;
	int16_t gyroChange = 0;
	
	pidStructInit(PTune, ITune, &pidStruct);
	
	for(;;){
		gyroValue = readGyroZ() - gyroOffset;
		
		gyroChange += (gyroValue/100); 
		
		if(gyroChange > turnComplete){
			break;
		}
		
		pidReturn = pidCalculation(3000, gyroValue, &pidStruct);//Will return a negative value if drifing left
		
		motorSpeedLeft(-pidReturn-3);
		motorSpeedRight(pidReturn);
		
		_delay_ms(10);
	}
}

void rightTurnGyro(){//Preform a left turn using the gyro
	int16_t gyroValue = 0;
	const int16_t gyroOffset = readGyroZ();
	const float PTune = 0.006;
	const float ITune = 0.00002;
	struct PID_STRUCT pidStruct;
	int16_t pidReturn;
	const int16_t turnComplete = -2300;
	int16_t gyroChange = 0;
	
	pidStructInit(PTune, ITune, &pidStruct);
	
	for(;;){
		gyroValue = readGyroZ() - gyroOffset;
		
		gyroChange += (gyroValue/100);
		
		if(gyroChange < turnComplete){
			break;
		}
		
		pidReturn = pidCalculation(-3000, gyroValue, &pidStruct);//Will return a negative value if drifing left
		
		motorSpeedLeft(-pidReturn + 3);
		motorSpeedRight(pidReturn);
		
		_delay_ms(10);
	}
}

void leftTurn(){
	uint8_t leftEncode = LEFT_ENCODER;
	uint8_t rightEncode = RIGHT_ENCODER;
	
	uint8_t leftTicks = 0;
	uint8_t rightTicks = 0;
	
	motorSpeedLeft(65,true);
	motorSpeedRight(67,false);
	
	//delayS(2);
	
	for(;;){
		if(leftEncode != LEFT_ENCODER){
			leftEncode = LEFT_ENCODER;
			leftTicks++;
			if(leftTicks == 3){
				motorSpeedLeft(0,false);	
			}
		}
		
		if(rightEncode != RIGHT_ENCODER){
			rightEncode = RIGHT_ENCODER;
			rightTicks++;
			if(rightTicks == 2){
				motorSpeedRight(0,false);
			}
		}
		
		if(leftTicks == 3 && rightTicks == 2){
			motorSpeedBoth(0,0);
			return;
		}
		
		_delay_ms(10);
	}
}

void rightTurn(){
	
}

void turn180(){
	
}

void north(uint8_t heading){
	
}

void east(uint8_t heading){
	
}

void south(uint8_t heading){
	
}

void west(uint8_t heading){
	
}



//Depreciated Code-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//void motorTicks(int tickMax,int8_t speed)//Moves wheels for specified ticks
//{
	//DDRC &= 0b11111100;
	//PORTC &= 0b11111100;
	////char status = (PINC && ENCODER_MASK);//Mask for PIND
	//int8_t leftStatus = LEFT_ENCODER;
	//int8_t rightStatus = RIGHT_ENCODER;
	//int leftTick = 0;
	//int rightTick = leftTick;
	//int8_t speedLeft = speed;
	//int8_t speedRight = speed * 0.5;
	////PC0 - Left        PC1 - Right
	////DDRC &= 0b11110011;//Makes sure DDRD is set right
	//motorSpeedBoth(speed,speed);
	//while((leftTick < tickMax) && (rightTick < tickMax))
	//{
		//if(LEFT_ENCODER != leftStatus)//Left ticks see if pin has changed
		//{
			//leftStatus = LEFT_ENCODER;//Reused mask
			//leftTick++;
		//}
		//
		//if(RIGHT_ENCODER != rightStatus)//Right ticks see if pin has changed
		//{
			//rightStatus = RIGHT_ENCODER;//Reused mask
			//rightTick++;
		//}
		//
		////if(leftTick < rightTick)motorSpeed(speed*.9,speed);
		////else if(leftTick > rightTick)motorSpeed(speed,speed*.9);
		//
		//if(leftTick < rightTick){
			//if(speedRight > 17)speedRight -= 1;
			//if(speedLeft < speed)speedLeft +=1;
			//
			//}else if(leftTick > rightTick ){
			//if(speedLeft > 17)speedLeft -= 1;
			//if(speedRight < speed)speedRight +=1;
		//}
		//
		//motorSpeedBoth(speedLeft,speedRight);
		////rightTick = 1;
		//_delay_ms(20);
	//}
	//
	//motorBrake(true, true);
//}
//
//
//void delay_mS(unsigned int mS)//Wait time in miliseconds
//{
	//while(mS > 0)
	//{
		//if(mS >= 1000)
		//{
			//_delay_ms(1000);
			//mS -= 1000;
		//}else if(mS >= 100)
		//{
			//_delay_ms(100);
			//mS -= 100;
		//}else if(mS >= 10)
		//{
			//_delay_ms(10);
			//mS -= 10;
		//}else
		//{
			//_delay_ms(1);
			//mS -= 1;
		//}
	//}
//}
//
//void motorTicksPerSecond(int leftTicksSec, int rightTicksSec){
	//const float PTune = 0.5;//This should be negative// P -0.5  I -0.001
	//const float ITune = 0;//0.15;// -0.0001;  0.5
	//float PCompL = 0;
	//float PCompR = 0;
	//float ICompL = 0;
	//float ICompR = 0 ;
	//float MeasuredTicksL = 0;
	//float MeasuredTicksR = 0;
	//int loopTime = 5;
	//int totalTimeMs = 0;
	//int lastTickMeasuredL = 0;
	//int lastTickMeasuredR = 0;
	//int measuredSpeedL = 0;
	//int measuredSpeedR = 0;
	//int motorSpeedL;
	//int motorSpeedR;
	//
	//float biasL = 25;//Free running this is ~ 5.5 ticks
	//float biasR = 25;
	//
	//int mSPerTickL = 1000/leftTicksSec;
	//int mSPerTickR = 1000/rightTicksSec;
	//
	//int8_t leftStatus = LEFT_ENCODER;
	//int8_t rightStatus = RIGHT_ENCODER;
//
	//motorSpeedLeft(biasL);
	//motorSpeedRight(biasR);
	//delay_mS(loopTime+1000);
	//totalTimeMs += loopTime;
	//
	//while(true){
		////totalTimeMs += loopTime;
		////Left PID Loop
		//if(leftStatus != LEFT_ENCODER || (totalTimeMs - lastTickMeasuredL) > mSPerTickL){
			//if(leftStatus != LEFT_ENCODER){
				//MeasuredTicksL++;
			//}
			//
			////measuredSpeedL = (measuredSpeedL + ((totalTimeMs - lastTickMeasuredL)))/2;//This computes the speed. It's averaged over two results.
			//measuredSpeedL = totalTimeMs - lastTickMeasuredL;
			//
			//PCompL = ((mSPerTickL - measuredSpeedL)) * PTune;
			////PCompL = ((measuredSpeedL - mSPerTickL)) * PTune;
			//
			//ICompL = ((totalTimeMs/mSPerTickL) - MeasuredTicksL) * ITune;
			//
			////motorSpeedL = PCompL + ICompL;
			//motorSpeedL = PCompL + ICompL;//biasL + 3;
			//
			//if(biasL >= 100)biasL = 99;
			//
			//if(motorSpeedL <= 15)motorSpeedL = 16;
			//
			////motorSpeedL = biasL;//Reviersices the motor as a fix for negative P
			//
			//motorSpeedLeft(motorSpeedL);
			//
			//
			//
			//if(leftStatus != LEFT_ENCODER){
				//lastTickMeasuredL = totalTimeMs;
				//leftStatus = LEFT_ENCODER;
			//}
		//}
		//
		////else if((totalTimeMs - lastTickMeasuredL) > mSPerTickL){//Same loop but no update		}
		//
		////Right PID Loop
		//if(rightStatus != RIGHT_ENCODER || (totalTimeMs - lastTickMeasuredR) > mSPerTickR){
			//if(rightStatus != RIGHT_ENCODER){
				//MeasuredTicksR++;
			//}
			//
			////measuredSpeedR = (measuredSpeedR + ((totalTimeMs - lastTickMeasuredR)))/2;//This computes the speed. It's averaged over two results.
			//measuredSpeedR = totalTimeMs - lastTickMeasuredR;
			//
			//PCompR = ((mSPerTickR - measuredSpeedR)) * PTune;
			////PCompR = ((measuredSpeedR - mSPerTickR)) * PTune;
			//
			//ICompR = ((totalTimeMs/mSPerTickR) - MeasuredTicksR) * ITune;
			//
			////motorSpeedR = PCompR + ICompR;
			//motorSpeedR = PCompR + ICompR + 6;// + 20;// + biasR;
			//
			//if(biasR >= 100)biasR = 99;
			//
			//if(motorSpeedR <= 15)motorSpeedR = 16;
			//
			//
			////motorSpeedR = biasR;//Reviersices the motor as a fix for negative P
			//
			//motorSpeedRight(motorSpeedR);
			//
			//if(rightStatus != RIGHT_ENCODER){
				//lastTickMeasuredR = totalTimeMs;
				//rightStatus = RIGHT_ENCODER;
			//}
		//}
		//
		//delay_mS(loopTime);
		//totalTimeMs += loopTime;
	//}
	//
	//
//}
//
////This PID code is based on Atmel's app note on PID. To find the app note google AN_2558. http://www.microchip.com/wwwappnotes/appnotes.aspx?appnote=en591227
//void moveStraight(int16_t distance_CM, int16_t CM_Second_SetPoint){//This is a PID algorithm for smooth and acurate motion control
	//distance_CM *= 100;//Converts 2.26 CM to 226
	//CM_Second_SetPoint *=100;
	////Wheel diameter was measured as 5.77 CM
	////#define WHEEL_CIRCUMFRENCE 18.1//Measure ticks per rotation
	////For the current hall effect sensors I count 4 high sections or 8 edges per-rotation
	////Distance per tick = 18.1/8 = 2.26 CM
	//int16_t msPerTick = ((int16_t)226*100)/(CM_Second_SetPoint/10);//This is simplifyed to avoid floats. In proper form it's 1000mS/(CM_Second_SetPoint/CM_Per_Tick)
	//const int16_t PTune = 2;
	////const float ITune = 0;
	//const int32_t distanceTick = 226;//2.26 CM  Casting only uses the LSB's meaning that a 32Bit int needs to be less than the max for a 16bit int, this is 32bit to temp convert a value to a big enough range
	//int16_t CM_Second_Left = 0, CM_Second_Right = 0;
	//struct PID_STRUCT pidStructL;
	//struct PID_STRUCT pidStructR;
	//
	////int16_t CM_Second_Measured;
	////int16_t error;
	//int16_t totalTimeMs = 0;
	//int16_t lastMeasurementLeft = 0, lastMeasurementRight = 0;
	//const int16_t loopTime = 5;
	//uint8_t leftStatus = LEFT_ENCODER;
	//uint8_t rightStatus = RIGHT_ENCODER;
	//int16_t leftPIDResult, rightPIDResult;
	//bool leftReverse = false, rightReverse = false;
	//uint8_t leftMotorSpeed, rightMotorSpeed;
	////error = CM_Second_SetPoint - CM_Second_Measured;// Error = set point - process value
	//#define blank 0
	//bool leftDone = false, rightDone = false;
	//
	//pidStructInit(PTune, &pidStructL);
	//pidStructInit(PTune, &pidStructR);
	//
	//motorSpeedBoth(29,25);//This is to get it started
	//_delay_ms(250);
	//
	//while(!leftDone || !rightDone){
		//if(!leftDone && leftStatus != LEFT_ENCODER){
			////CM_Second_Left = ((totalTimeMs - lastMeasurementLeft)/1000) * distanceTick;
			////CM_Second_Left = (int16_t)(((totalTimeMs - lastMeasurementLeft) * ((int32_t)distanceTick))/1000);//45.2 -> 0.45 CM/S
			//CM_Second_Left = ((int16_t)(((totalTimeMs - lastMeasurementLeft) * distanceTick)/1000) + CM_Second_Left)/2;//45.2 -> 0.45 CM/S
			//
			//leftPIDResult = pidCalculation(CM_Second_SetPoint , CM_Second_Left, &pidStructL);
			//
			//if(leftPIDResult < 0){
				////leftReverse = true;
				//leftPIDResult = 10 << 8;
				//}else{
				//leftReverse = false;
			//}
			//
			////leftMotorSpeed = (leftPIDResult >> 7) * 0xFF;
			//
			//motorSpeedLeft(leftMotorSpeed + blank + 3, leftReverse);
			//
			//lastMeasurementLeft = totalTimeMs;
			//leftStatus = LEFT_ENCODER;
		//}
		//msPerTick = 300;
		//if(rightStatus != RIGHT_ENCODER || (lastMeasurementRight + msPerTick) < totalTimeMs){
			////if(rightStatus != RIGHT_ENCODER){
			//if(rightStatus != RIGHT_ENCODER){
				////CM_Second_Right = ((totalTimeMs - lastMeasurementRight)/1000.0) * distanceTick;//One floating point part, almost eliminated it!
				//CM_Second_Right = (1000.0/(totalTimeMs - lastMeasurementRight)) * distanceTick;//One floating point part, almost eliminated it!
				//}else{
				////CM_Second_Right = (((totalTimeMs + msPerTick)- lastMeasurementRight)/1000.0) * distanceTick;
				//CM_Second_Right = (1000.0/(totalTimeMs - lastMeasurementRight)) * distanceTick;
			//}
			////CM_Second_Right = ((int16_t)(((totalTimeMs - lastMeasurementRight) * distanceTick)/1000) + CM_Second_Right)/2;//45.2 -> 0.45 CM/S   Also casting only uses the LSB's meaning that a 32Bit int needs to be less than the max for a 16bit int
			////CM_Second_SetPoint = 1000;
			//uint16_t CM_Second_SetPoint2 = 1000;
			//uint8_t left = 0x03, right = 0xE8;
			//
			//I2CStart(0x02);
			//I2CDataSend(totalTimeMs >> 8);
			//I2CDataSend(totalTimeMs & 0xFF);
			//I2CDataSend(lastMeasurementRight >> 8);
			//I2CDataSend(lastMeasurementRight & 0xFF);
			//
			//I2CDataSend(CM_Second_Right >> 8);
			//I2CDataSend(CM_Second_Right & 0xFF);
			////I2CDataSend(CM_Second_SetPoint >> 8);
			////I2CDataSend(CM_Second_SetPoint & 0xFF);
			////I2CDataSend(left);
			////I2CDataSend(right);
			//I2CStop();
			//
			//rightPIDResult = pidCalculation(CM_Second_SetPoint , CM_Second_Right, &pidStructR);
			//
			//if(rightPIDResult < 0){
				////rightReverse = true;
				//rightPIDResult = 10 << 8;
				//}else{
				//rightReverse = false;
			//}
			//
			//rightMotorSpeed = (rightPIDResult >> 7) & 0xFF;
			//
			//motorSpeedRight(rightMotorSpeed + blank, rightReverse);
			//
			//I2CStart(0x02);
			//I2CDataSend(rightPIDResult >> 8);
			//I2CDataSend(rightPIDResult & 0xFF);
			//I2CDataSend(rightMotorSpeed);
			//I2CStop();
			//
			////motorSpeedRight(pidCalculation(CM_Second_SetPoint , CM_Second_Right, pidStructR), false);
			//
			//if(rightStatus != RIGHT_ENCODER){
				//lastMeasurementRight = totalTimeMs;
				//rightStatus = RIGHT_ENCODER;
			//}
		//}
		//_delay_ms(loopTime);
		//totalTimeMs += loopTime;
	//}
//}
//

//void moveStraight2(){
	////motorSpeed(25,25);
	//#define speedL 63
	//#define speedR 70
	////motorSpeed(speedL,speedR);
	//motorSpeedLeft(speedL,false);
	//motorSpeedRight(speedR,false);
	//
	////for(;;);
	//
	//#define CLOSE_R 4000
	//#define NORMAL_R 3200
	//#define FAR_R 4000
	//
	//#define CLOSE_L CLOSE_R
	//#define NORMAL_L NORMAL_R
	//#define FAR_L FAR_R
	//
	//int16_t distanceL = 0;
	//int16_t distanceR = 0;
	//int16_t distanceFront = 0;
	//float error = 0;
	//float errorTemp = 0;
	//float lastError = 0;
	//float sumError = 0;
	//int16_t p_term = 0;
	//int16_t i_term = 0;
	//int16_t dTerm = 0;
	//uint8_t frontDelay = 0;
	//#define ADJUST 3
	//
	////for(;;){
	////	_delay_ms(10);
	////}
	//
	//for(;;){
		//distanceL = readIR(IR_LEFT);
		//distanceR = readIR(IR_RIGHT);
		//distanceFront = readIR(IR_FRONT);
		////error = distanceR - distanceL;
		////error *= 0.05;
		//
		//
		////if(distanceR > FAR_R && distanceL > FAR_L){
		//distanceL += 600;//2000;//7000;//14000
		//errorTemp = distanceR - distanceL;
		////error = log(abs(errorTemp));
		//
		//if(errorTemp < 0){
			//error = log(errorTemp * -1);
			//error *= -1;
			//}else{
			//error = log(errorTemp);
		//}
		//
		//p_term = error * 7.0;//0.01;//0.005;//0.003
		//
		////error -= 5;
		//
		////}else if(distanceR > FAR_R){//No wall left
		////error = distanceR - 3000;
		////error *= 0.05;
		////}else if(distanceL > FAR_L){//No wall right
		////error = NORMAL_R - 3000;
		////error *= 0.05;
		////}else{
		////break;
		////}
		//
		//#define P_MAX 20
		//
		//if(p_term > P_MAX){
			//p_term = P_MAX;
			//}else if(p_term < -P_MAX){
			//p_term = -P_MAX;
		//}
		//
		//dTerm = 0.0 * (lastError - error);
		//
		//if(error > 10){
			////error = 0;
			//}else if(error < 10){
			////error = -10;
		//}
		//
		//sumError = sumError + error;
		//i_term = sumError * 0.0;
		//
		//if ( speedR + error < 0) error = -speedR;
		//motorSpeedRight((speedR + p_term) + dTerm + i_term, false);
		//if ( speedL - error < 0) error = speedL;
		////motorSpeedLeft((speedL - p_term) - dTerm - i_term,false);
		//
		//lastError = error;
		//
		//if(distanceFront > 3500){
			//frontDelay++;
			//}else{
			//frontDelay = 0;
		//}
		//
		//if(frontDelay > 5){
			////motorBrake(true,true);
			////return;
		//}
		//
		//
		//
		//_delay_ms(10);
	//}
//}