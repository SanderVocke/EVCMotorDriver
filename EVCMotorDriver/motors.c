/*
 * motors.c
 *
 * Created: 4-5-2015 14:18:29
 *  Author: Sander
 */ 

#include "motors.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

void initMotors(void){
	//control globals: measurement
	intref_left[0] = intref_left[1] = intref_left[2] = 0;
	intref_right[0] = intref_right[1] = intref_right[2] = 0;
	lastangle_right = 0;
	lastangle_left = 0;
	do_ticks=1;
	ticks_right = 0;
	ticks_left = 0;
	prescaler = DIV_SPEED_TIMER;
	doUpdate = 0;
	#ifdef STORE_ANGLES
	i_angles_left = 0;
	i_angles_right = 0;
	#endif

	//control globals: PID
	t_speed_left = 0;
	t_speed_right = 0;
	speed_left = 0;
	speed_right = 0;
	duty_left = 0;
	duty_right = 0;
	left_dir = FORWARD;
	right_dir = FORWARD;
	I_left = 0;
	I_right = 0;
	
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
	setMotorDirection(RIGHT_MOTOR, BACKWARD);
	setMotorDuty(LEFT_MOTOR, 0);
	setMotorDuty(RIGHT_MOTOR, 0);
#ifndef OPEN_LOOP_CONTROL
	setMotorSpeed(LEFT_MOTOR, 0);
	setMotorSpeed(RIGHT_MOTOR, 0);
#endif
	
	//Init Timers	
	//Timer/Counter1: For 100kHz PWM to motor driver.
	TCCR1A = 0b11110001; //Fast PWM mode, inverting
	TCCR1B = 0b00001001; //Fast PWM mode, system clock source	
	
	#ifndef OPEN_LOOP_CONTROL //if doing PID
	//Init external interrupts (PWM inputs)
	MCUCR |= 0b00000101; //any edge causes interrupt
	GICR |= 0b11000000; //enable INTO and INT1
	TCNT0 = 0; //reset timer 0 (right wheel)
	TCCR0 = 0b00000010; //clk/8 = almost 500Hz overflow rate for the timer that times PWM. Starts timer.
	TIMSK |= 0b00000001; //enable overflow interrupt
	#endif
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

uint8_t getMotorSpeed(motor_t motor){
	switch(motor){
		case LEFT_MOTOR:
		return t_speed_left;
		break;
		case RIGHT_MOTOR:
		return t_speed_right;
		break;
	}	
	return 0;
}

uint8_t getMotorDuty(motor_t motor){
	return (motor==LEFT_MOTOR) ? duty_left : duty_right;
}

void setMotorDuty(motor_t motor, uint8_t duty){
	switch(motor){
		case LEFT_MOTOR:
		duty_left = duty;
		OCR1A = 255-duty;
		break;
		case RIGHT_MOTOR:
		duty_right = duty;
		OCR1B =255-duty;
		break;
	}
}

direction_t getMotorDirection(motor_t motor){
	switch(motor){
		case LEFT_MOTOR:
		return (PORTA & 0b00000001) ? FORWARD : BACKWARD;
		break;
		case RIGHT_MOTOR:
		return (PORTA & 0b00000010) ? BACKWARD : FORWARD;
		break;
	}
	return FORWARD;
}

void setMotorDirection(motor_t motor, direction_t direction){
	switch(motor){
		case LEFT_MOTOR:
		if(direction != left_dir){
			setMotorDuty(LEFT_MOTOR, 0); //stop if changing direction
			I_left = 0; //reset error
		}
		if(direction==FORWARD) PORTA |= 0b00000001;
		else PORTA &= 0b11111110;
		left_dir = direction;
		break;
		case RIGHT_MOTOR:
		if(direction != right_dir){
			setMotorDuty(RIGHT_MOTOR, 0); //stop if changing direction
			I_right = 0; //reset error
		}
		if(direction==FORWARD) PORTA &= 0b11111101;
		else PORTA |= 0b00000010;
		right_dir = direction;
		break;
	}
}

void doPID(){
	if(t_speed_left <= MIN_SPEED) setMotorDuty(LEFT_MOTOR, 0);
	else{
		error = (int32_t)t_speed_left - (int32_t)speed_left;
		I_left += error;
		if(I_left > I_CAP) I_left = I_CAP;
		if(I_left < (-I_CAP)) I_left = -I_CAP;
		newSpeed = (P_GAIN*error)/1000 + (I_GAIN*I_left)/1000 + (D_GAIN*(error-lasterror_left))/1000 + (int16_t)getMotorDuty(LEFT_MOTOR);
		if(newSpeed < 0) newSpeed = 0;
		if(newSpeed > 255) newSpeed = 255;
		setMotorDuty(LEFT_MOTOR, (uint8_t)newSpeed);
		lasterror_left = error;
	}
	
	if(t_speed_right <= MIN_SPEED) setMotorDuty(RIGHT_MOTOR, 0);
	else{
		error = (int32_t)t_speed_right - (int32_t)speed_right;
		I_right += error;
		if(I_right > I_CAP) I_right = I_CAP;
		if(I_right < (-I_CAP)) I_right = -I_CAP;
		newSpeed = (P_GAIN*error)/1000 + (I_GAIN*I_right)/1000 + (D_GAIN*(error-lasterror_right))/1000 + (int16_t)getMotorDuty(RIGHT_MOTOR);
		if(newSpeed < 0) newSpeed = 0;
		if(newSpeed > 255) newSpeed = 255;
		setMotorDuty(RIGHT_MOTOR, (uint8_t)newSpeed);
		lasterror_right = error;
	}
	return;
}

//function that updates speed
void updateSpeed(void){
	if(!doUpdate) return;
	doUpdate = 0;
	uint16_t speed16r = 10*ticks_right;
	uint16_t speed16l = 10*ticks_left;
	ticks_right = 0;
	ticks_left = 0;
	speed_left = (speed16l > 255)?255:(uint8_t)speed16l;
	speed_right = (speed16r > 255)?255:(uint8_t)speed16r;
	doPID();
}

//interrupt handlers for external interrupts (get angle readings from motors)
ISR(INT0_vect){ //right motor
	uint8_t sreg_save = SREG;
	cli();
	uint8_t current_value = TCNT0;
	if(PIND & 0b00000100){ //just went HIGH
		uint8_t temp = intref_right[1]-intref_right[0];
		uint8_t temp2 = current_value-intref_right[1];
		uint16_t temp4 = temp2+temp;
		uint16_t temp3 = ((uint16_t)temp<<8)/(uint16_t)temp4; //angle
		if(do_ticks){
			if(temp3 < 170 && lastangle_right > 170) ticks_right++;
		}
		lastangle_right = temp3;
		#ifdef STORE_ANGLES
		angles_right[i_angles_right%NUM_STORE_ANGLES] = lastangle_right;
		i_angles_right++;
		#endif
		intref_right[0] = current_value;
	}
	else{ //just went LOW
		intref_right[1] = current_value;
	}
	SREG = sreg_save;
}
ISR(INT1_vect){ //left motor
	uint8_t sreg_save = SREG;
	cli();
	uint8_t current_value = TCNT0;
	if(PIND & 0b00001000){ //just went HIGH
		uint8_t temp = intref_left[1]-intref_left[0];
		uint8_t temp2 = current_value-intref_left[1];
		uint16_t temp4 = temp2+temp;
		uint16_t temp3 = ((uint16_t)temp<<8)/(uint16_t)temp4; //angle
		if(do_ticks){
			if(temp3 < 170 && lastangle_left > 170) ticks_left++;
		}
		lastangle_left = temp3;
		#ifdef STORE_ANGLES
		angles_left[i_angles_left%NUM_STORE_ANGLES] = lastangle_left;
		i_angles_left++;
		#endif
		intref_left[0] = current_value;
	}
	else{ //just went LOW
		intref_left[1] = current_value;
	}
	SREG = sreg_save;
}

//Overflow handler of timer 0 (500Hz)
ISR(TIMER0_OVF_vect){
	if(prescaler) prescaler--;
	else{
		prescaler = DIV_SPEED_TIMER;
		doUpdate = 1;
	}
}