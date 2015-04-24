/*
 * EVCMotorDriver.c
 *
 * Created: 23-4-2015 19:29:49
 *  Author: Sander
 */ 


#include <avr/io.h>

#define SYSTEMCLK 1000000 //1MHz setting (make sure this is set for the device!)

typedef enum motor_t{LEFT_MOTOR = 0, RIGHT_MOTOR}motor_t;
typedef enum direction_t{FORWARD = 0, BACKWARD}direction_t;
	
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
	setMotorDirection(LEFT_MOTOR, FORWARD);
	setMotorDirection(RIGHT_MOTOR, FORWARD);
	
	//Init Timers	
	//Timer/Counter1: For 100kHz PWM to motor driver.
	TCCR1A = 0b11110001; //Fast PWM mode, inverting
	TCCR1B = 0b00001001; //Fast PWM mode, system clock source	
}