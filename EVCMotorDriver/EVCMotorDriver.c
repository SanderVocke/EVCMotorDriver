/*
 * EVCMotorDriver.c
 *
 * Created: 23-4-2015 19:29:49
 *  Author: Sander
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

//////////////////////////////////////
// DEFINITIONS
//////////////////////////////////////

//control settings
//#define OPEN_LOOP_CONTROL //disables PID and does direct control instead
#define NUM_ANGLE_PER_SPEED 3 //amount of angle readings to determine the speed over
#define SPEED_MULT 2048
#define SPEED_UNDER_THRESHOLD 20

//bit flipping
#define SET(x,y) (x|=(1<<y))
#define CLR(x,y) (x&=(~(1<<y)))
#define CHK(x,y) (x&(1<<y))
#define TOG(x,y) (x^=(1<<y))

#define SYSTEMCLK 1000000 //1MHz setting (make sure this is set for the device!)
#define I2C_SLAVE_ADDR 0x0B //I2C slave address
#define DIRECTION_BIT 0x01 //direction selector
#define MOTOR_BIT 0x02 //motor selector

//I2C communication defines:
//ACK the next transmission
//and indicate that we've handled the last one.
#define TWACK (TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA)|(1<<TWIE))
//NACK the next transmission
#define TWNACK (TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWIE))
//reset the I2C hardware (used when the bus is in a illegal state)
#define TWRESET (TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO)|(1<<TWEA)|(1<<TWIE))

typedef enum motor_t{LEFT_MOTOR = 0, RIGHT_MOTOR}motor_t;
typedef enum direction_t{FORWARD = 0, BACKWARD}direction_t;
	
//I2C
#define I2C_BUFSIZE_RECV 6
#define I2C_BUFSIZE_TRAN 3
#define I2C_CLEARINT {TWCR |= (1<<TWINT);}
	
//////////////////////////////////////
// GLOBALS
//////////////////////////////////////

//control globals: measurement
uint8_t inttimes_left[NUM_ANGLE_PER_SPEED*2][2];
uint8_t inttimes_right[NUM_ANGLE_PER_SPEED*2][2];
uint8_t intref_left = 0;
uint8_t intref_right = 0;
uint8_t intindex_left = 0;
uint8_t intindex_right = 0;
uint8_t do_right = 0;
uint8_t do_left = 0;
uint8_t angles_right[NUM_ANGLE_PER_SPEED];
uint8_t speed_right = 0;
uint8_t angles_left[NUM_ANGLE_PER_SPEED];
uint8_t speed_left = 0;

//control globals: PID
uint8_t t_speed_left = 0;
uint8_t t_speed_right = 0;

//I2C globals
uint8_t i2c_r_index = 0;
uint8_t i2c_r[I2C_BUFSIZE_RECV]; //buffer to store received bytes
uint8_t i2c_rst = 0; //if something fails...
uint8_t i2c_t_index=0;
uint8_t i2c_t[I2C_BUFSIZE_TRAN] = {0, 0, 0};
	
//globals to keep track of things between bytes
motor_t i2c_motor = LEFT_MOTOR;
direction_t i2c_direction = FORWARD;
	
//Prototypes
void setMotorDuty(motor_t motor, uint8_t duty);
void setMotorDirection(motor_t motor, direction_t direction);
void setMotorSpeed(motor_t motor, uint8_t speed);
void doPID(motor_t motor);
void updateSpeed(void);
void init(void);

int main(void)
{
	init();
	
    while(1){
#ifndef OPEN_LOOP_CONTROL
		updateSpeed();
#endif
    }
}

void setMotorSpeed(motor_t motor, uint8_t speed){
	switch(motor){
		case LEFT_MOTOR:
		t_speed_left = speed;
		break;
		case RIGHT_MOTOR:
		t_speed_right = speed;
		break;
	}
}

uint8_t getMotorDuty(motor_t motor){
	return (motor==LEFT_MOTOR) ? 255-OCR1A : 255-OCR1B;
}

void setMotorDuty(motor_t motor, uint8_t duty){
	switch(motor){
		case LEFT_MOTOR:
		OCR1A = 255-duty;
		break;
		case RIGHT_MOTOR:
		OCR1B =255-duty;
		break;
	}
}

void setMotorDirection(motor_t motor, direction_t direction){
	switch(motor){
		case LEFT_MOTOR:
		if(direction==FORWARD) PORTA |= 0b00000001;
		else PORTA &= 0b11111110;
		break;
		case RIGHT_MOTOR:
		if(direction==FORWARD) PORTA &= 0b11111101;
		else PORTA |= 0b00000010;
		break;
	}
}

void init(void){
	//Init I/O pins
	
	/* Port A:
	- pins 0 and 1: direction outputs to motor controller (CW/CCW).
	*/
	DDRA |= 0b00000011;
	
	/* Port D:
	- pins 4 and 5: PWM outputs to motor controller.
	- pins 2 and 3: PWM (interrupt) inputs from angle sensors.
	*/
	DDRD |= 0b00110000;
	
	//Init Motor Parameters
	setMotorDirection(LEFT_MOTOR, FORWARD);
	setMotorDirection(RIGHT_MOTOR, FORWARD);
	setMotorDuty(LEFT_MOTOR, 0);
	setMotorDuty(RIGHT_MOTOR, 0);
#ifndef OPEN_LOOP_CONTROL
	setMotorSpeed(LEFT_MOTOR, 0);
	setMotorSpeed(RIGHT_MOTOR, 100);
#endif
	
	//Init Timers	
	//Timer/Counter1: For 100kHz PWM to motor driver.
	TCCR1A = 0b11110001; //Fast PWM mode, inverting
	TCCR1B = 0b00001001; //Fast PWM mode, system clock source	
	
	//Init I2C slave mode
	TWAR = (I2C_SLAVE_ADDR<<1)|1; //slave addr (bits 7-1) and respond to general call
	TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE); //slave mode config
	
#ifndef OPEN_LOOP_CONTROL //if doing PID
	//Init external interrupts (PWM inputs)
	MCUCR |= 0b00000101; //any edge causes interrupt
	GICR |= 0b11000000; //enable INTO and INT1
	TCNT0 = 0; //reset timer 0 (right wheel)
	TCCR0 = 0b00000010; //clk/8 = almost 500Hz overflow rate for the timer that times PWM. Starts timer.	
#endif
	
	//global interrupts
	sei();
}

void processI2CByte(){
	uint8_t byte = TWDR;
	//first byte: selects motor and direction.	
	if(i2c_r_index==0){
		i2c_motor = (byte & MOTOR_BIT) ? LEFT_MOTOR : RIGHT_MOTOR;
		i2c_direction = (byte & DIRECTION_BIT) ? FORWARD : BACKWARD;		
	}
	else if(i2c_r_index==1){
		setMotorDirection(i2c_motor, i2c_direction);
#ifdef OPEN_LOOP_CONTROL
		setMotorDuty(i2c_motor, byte);
//#else (closed-loop control)
#endif
	}
}

int32_t I_left = 0;
int32_t I_right = 0;
#define P_GAIN 10 //of 100
#define I_GAIN 0 //of 100
#define I_CAP 1000
void doPID(motor_t motor){
	int16_t error;
	if(motor == LEFT_MOTOR){
		error = (int32_t)t_speed_left - (int32_t)speed_left;
		I_left += error;
		int16_t newSpeed = (P_GAIN*error)/1000 + (I_GAIN*I_left)/1000 + (int16_t)getMotorDuty(LEFT_MOTOR);
		if(newSpeed < 0) newSpeed = 0;
		if(newSpeed > 255) newSpeed = 255;
		setMotorDuty(LEFT_MOTOR, (uint8_t)newSpeed);
	}
	else{
		error = (int32_t)t_speed_right - (int32_t)speed_right;
		I_right += error;
		if(I_right > I_CAP) I_right = I_CAP;
		int16_t newSpeed = (P_GAIN*error)/1000 + (I_GAIN*I_right)/1000 + (int16_t)getMotorDuty(RIGHT_MOTOR);
		if(newSpeed < 0) newSpeed = 0;
		if(newSpeed > 255) newSpeed = 255;
		setMotorDuty(RIGHT_MOTOR, (uint8_t)newSpeed);
	}
	return;	
}

//function that calculates speed from measurements
uint8_t calculateSpeed(uint8_t inttimes[][2], uint8_t*angles){
	uint16_t total_time=0;
	uint8_t i;
	for(i=0; i<NUM_ANGLE_PER_SPEED; i++){ //calculate the last set of angles
		uint16_t high = (uint16_t)inttimes[i][0];
		uint16_t total = high + (uint16_t)inttimes[i][1];
		total_time += total;
		angles[i] = (uint8_t)((high<<8)/total); //angle in 1/256 of radians (roughly)
	}
	//now, aggregate the angles
	uint16_t total_angle=0;
	
	for(i=0; i<NUM_ANGLE_PER_SPEED-1; i++){
		int16_t diff = (int16_t)angles[i+1] - (int16_t)angles[i];
		if(diff<0) diff = -diff;
		if(diff>128) diff = 256-diff;
		total_angle += (uint16_t)diff;
	}
	//finally, calculate the speed.
	uint16_t speed16 = total_angle*((SPEED_MULT)/(NUM_ANGLE_PER_SPEED*total_time));
	if(speed16<SPEED_UNDER_THRESHOLD) return 0;
	return (speed16>255) ? 255 : (uint8_t) speed16;	
}

//function that updates speed
void updateSpeed(void){	
	if(!do_right){ //if do_right is 0, we should update the right PID
		speed_right = calculateSpeed(intindex_right==0 ? &(inttimes_right[NUM_ANGLE_PER_SPEED]): inttimes_right, angles_right);
		doPID(RIGHT_MOTOR);
		do_right = 1; //start counting again
	}
	if(!do_left){ //if do_right is 0, we should update the right PID
		speed_left = calculateSpeed(intindex_left==0 ? &(inttimes_left[NUM_ANGLE_PER_SPEED]): inttimes_left, angles_left);
		doPID(LEFT_MOTOR);
		do_left = 1; //start counting again
	}
}

//interrupt handlers for external interrupts (get angle readings from motors)
ISR(INT0_vect){ //right motor
	uint8_t sreg_save = SREG;
	cli();
	uint8_t current_value = TCNT0;
	if(PIND & 0b00000100){ //just went HIGH
		//save value
		inttimes_right[intindex_right][1] = (current_value - intref_right);
		if(do_right) {
			intindex_right++; //move to next only if currently active
			if(intindex_right%NUM_ANGLE_PER_SPEED == 0) do_right = 0; //deactivate if we did enough measurements for next sample
		}
	}
	else{ //just went LOW
		inttimes_right[intindex_right][0] = current_value - intref_right; //save value
	}
	intref_right = current_value;
	SREG = sreg_save;
}
ISR(INT1_vect){ //left motor
	uint8_t sreg_save = SREG;
	cli();
	uint8_t current_value = TCNT0;
	if(PIND & 0b00000100){ //just went HIGH
		//save value
		inttimes_left[intindex_left][1] = (current_value - intref_left);
		if(do_left) {
			intindex_left++; //move to next only if currently active
			if(intindex_left%NUM_ANGLE_PER_SPEED == 0) do_left = 0; //deactivate if we did enough measurements for next sample
		}
	}
	else{ //just went LOW
		inttimes_left[intindex_left][0] = current_value - intref_left; //save value
	}
	intref_left = current_value;
	SREG = sreg_save;
}

//interrupt handler for I2C slave operation
ISR(TWI_vect){	
	uint8_t sreg_save = SREG;
	cli();
	switch(TW_STATUS){
		//--------------Slave receiver------------------------------------
		//SLA_W received and acked, prepare for data receiving
		case 0x60:
		TWACK;
		i2c_r_index = 0;
		break;
		case 0x80:  //a byte was received, store it and
		//setup the buffer to recieve another
		//i2c_r[i2c_r_index] = TWDR;
		processI2CByte();
		i2c_r_index++;
		//don't ack next data if buffer is full
		if(i2c_r_index >= I2C_BUFSIZE_RECV){
			TWNACK;
			}else {
			TWACK;
		}
		break;
		case 0x68://adressed as slave while in master mode.
		//should never happen, better reset;
		i2c_rst=1;
		case 0xA0: //Stop or rep start, reset state machine
		TWACK;
		break;
		//-------------- error recovery ----------------------------------
		case 0x88: //data received  but not acked
		//should not happen if the master is behaving as expected
		//switch to not adressed mode
		TWACK;
		break;
		//---------------Slave Transmitter--------------------------------
		case 0xA8:  //SLA R received, prep for transmission
		//and load first data
		i2c_t_index=1;
		TWDR = i2c_t[0];
		TWACK;
		break;
		case 0xB8:  //data transmitted and acked by master, load next
		TWDR = i2c_t[i2c_t_index];
		i2c_t_index++;
		//designate last byte if we're at the end of the buffer
		if(i2c_t_index >= I2C_BUFSIZE_TRAN) TWNACK;
		else TWACK;
		break;
		case 0xC8: //last byte send and acked by master
		//last bytes should not be acked, ignore till start/stop
		//reset=1;
		case 0xC0: //last byte send and nacked by master
		//(as should be)
		TWACK;
		break;
		//--------------------- bus error---------------------------------
		//illegal start or stop received, reset the I2C hardware
		case 0x00:
		TWRESET;
		break;
	}
	SREG = sreg_save;
}