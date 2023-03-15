/*
 * Motor.h
 * Class used control a motor
 *
 *  Created on: 20 Mar 2022
 *      Author: jonkemp
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "Arduino.h"
#include "PID_v2.h"

enum SIDE {
	LEFT,
	RIGHT
};

// Motor PWM hardware constants
#define speedPinR 5    			// RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1  7   	// Right Motor direction pin 1 to MODEL-X IN1
#define RightDirectPin2  8    	// Right Motor direction pin 2 to MODEL-X IN2

#define speedPinL 6    		 // Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1  9    // Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2  10   // Left Motor direction pin 1 to MODEL-X IN4

#define MIN_DRIVE_VALUE -255	// Minimum PWM value (actually always +ve!)
#define MAX_DRIVE_VALUE  255	// Max PWM value

// PID tuning parameter constants
#define SPEED_KP	1.00		// Speed PID proportional
#define SPEED_KI	0.50		// Integral
#define SPEED_KD	0.00		// Differential

// Direction constants
#define STOPPED 0
#define FORWARD 1
#define REVERSE -1

// SPEEDSCALER is 1% of maximum pulseCounts/s and is used to calculate target speed
#define MAX_PULSES_PER_SEC 800		// Derived from testing motor at max PWM value
#define SPEEDSCALER MAX_PULSES_PER_SEC/100


class Motor {
public:
	Motor(SIDE);		// Constructor needs a specified motor
	~Motor();
	void setup();
	volatile long distanceCount = 0;	// Up/down count of pulses from motor = distance driven
	void setSpeed(int);
	void drive(int);		// Drive motor at a speed -100 - 0 +100
	long getDistance();		// Return distance count
	double getCurrentSpeed();

private:
	byte _speedPin;		// PWM pin for this motor
	byte _dirPin1;		// direction Pin1 for this motor
	byte _dirPin2;		// direction Pin2 for this moto
	byte _pulsePin;		// Pin attached to interrupt routine to count pulses

	int _cmdSpeed = 0;		// Desired motor speed -100 - 0 +100

	double _requiredSpeed = 0;	// Target speed pulses/s
	double _currentSpeed = 0;	// Actual speed pulses/s
	double _driveValue = 0; 	// Value sent to PWM 0 - 255
	byte _currentDir = STOPPED;
	byte _prevDir = STOPPED;

	// Vars to calculate time and distance changes
	unsigned long _prevMicros = 0;
	long _prevDistanceCount = 0;

	PID_v2 _speedPID;

	void _pulseHandler();	// Interrupt handler attached to _pulsePin;


};

#endif /* MOTOR_H_ */
