/*
 * motors.h
 *
 * Created: 4-5-2015 14:18:45
 *  Author: Sander
 */ 


#ifndef MOTORS_H_
#define MOTORS_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

//control settings
//#define OPEN_LOOP_CONTROL //disables PID and does direct control instead
#define DIV_SPEED_TIMER 40
#define MIN_SPEED 20
//#define STORE_ANGLES
#define NUM_STORE_ANGLES 32
#define P_GAIN 800 //of 1000
#define I_GAIN 100 //of 1000
#define D_GAIN 300 //of 1000
#define I_CAP 1000

typedef enum motor_t{LEFT_MOTOR = 0, RIGHT_MOTOR}motor_t;
typedef enum direction_t{FORWARD = 0, BACKWARD}direction_t;
	
//control globals: measurement
uint8_t intref_left[3];
uint8_t intref_right[3];
uint16_t lastangle_right;
uint16_t lastangle_left;
uint8_t do_ticks;
uint8_t ticks_right;
uint8_t ticks_left;
uint8_t doUpdate;
uint8_t prescaler;
#ifdef STORE_ANGLES
uint16_t angles_left[NUM_STORE_ANGLES];
uint8_t i_angles_left;
uint16_t angles_right[NUM_STORE_ANGLES];
uint8_t i_angles_right;
#endif

//control globals: PID
uint8_t t_speed_left;
uint8_t t_speed_right;
uint8_t speed_left;
uint8_t speed_right;
uint8_t duty_left;
uint8_t duty_right;
direction_t left_dir;
direction_t right_dir;
int32_t I_left;
int32_t I_right;
int32_t error, lasterror_right, lasterror_left, newSpeed;

//Prototypes
void setMotorDirection(motor_t motor, direction_t direction);
direction_t getMotorDirection(motor_t motor);
void doPID();
void updateSpeed(void);
void initMotors(void);
void setMotorDuty(motor_t motor, uint8_t duty);
void setMotorSpeed(motor_t motor, uint8_t speed);

#endif /* MOTORS_H_ */