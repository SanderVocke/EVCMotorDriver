/*
 * EVCMotorDriver.c
 *
 * Created: 23-4-2015 19:29:49
 *  Author: Sander
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

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

//I2C globals
#define I2C_BUFSIZE_RECV 6
uint8_t i2c_r_index = 0;
uint8_t i2c_r[I2C_BUFSIZE_RECV]; //buffer to store received bytes
uint8_t i2c_rst = 0; //if something fails...
#define I2C_BUFSIZE_TRAN 3
uint8_t i2c_t_index=0;
uint8_t i2c_t[I2C_BUFSIZE_TRAN] = {0, 0, 0};
	
//I2C operations
#define I2C_CLEARINT {TWCR |= (1<<TWINT);}
	
//globals to keep track of things between bytes
motor_t i2c_motor = LEFT_MOTOR;
direction_t i2c_direction = FORWARD;


	
//Prototypes
void setMotorDuty(motor_t motor, uint8_t duty);
void setMotorDirection(motor_t motor, direction_t direction);
void init(void);

int main(void)
{
	init();
	
    while(1)
    {
        
    }
}

void setMotorDuty(motor_t motor, uint8_t duty){
	switch(motor){
		case LEFT_MOTOR:
		OCR1A = 255-duty;
		break;
		case RIGHT_MOTOR:
		OCR1B = 255-duty;
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
	*/
	DDRD |= 0b00110000;
	
	//Init Motor Parameters
	setMotorDuty(LEFT_MOTOR, 0);
	setMotorDuty(RIGHT_MOTOR, 0);
	setMotorDirection(LEFT_MOTOR, BACKWARD);
	setMotorDirection(RIGHT_MOTOR, BACKWARD);
	
	//Init Timers	
	//Timer/Counter1: For 100kHz PWM to motor driver.
	TCCR1A = 0b11110001; //Fast PWM mode, inverting
	TCCR1B = 0b00001001; //Fast PWM mode, system clock source	
	
	//Init I2C slave mode
	TWAR = (I2C_SLAVE_ADDR<<1)|1; //slave addr (bits 7-1) and respond to general call
	TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE); //slave mode config
	
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
		setMotorDuty(i2c_motor, byte);
	}
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