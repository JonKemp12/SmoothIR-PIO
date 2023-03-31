/*
 * Motor.cpp
 * All the functions to control a motor's movements
 *
 *  Created on: 20 Mar 2022
 *      Author: jonkemp
 */

#include "Motor.h"

Motor::Motor(SIDE side) {
	_sideInstance = side;			// Identify which side instance this is.
	SideVars *sidePtr = &sides[side];
		sidePtr->distanceCount = 0;
		sidePtr->currentDir = 0;
		sidePtr->prevDir = 0;
	if (side == LEFT) {
		_speedPin = speedPinL;
		_dirPin1 = LeftDirectPin1;
		_dirPin2 = LeftDirectPin2;
		sidePtr->pulsePin = LeftPulsePin;
	} else {
		_speedPin = speedPinR;
		_dirPin1 = RightDirectPin1;
		_dirPin2 = RightDirectPin2;
		sidePtr->pulsePin = RightPulsePin;
	}

	// Initialise motor IO pinModes
	pinMode(_dirPin1, OUTPUT);
	pinMode(_dirPin2, OUTPUT);
	pinMode(_speedPin, OUTPUT);
	pinMode(sidePtr->pulsePin, INPUT_PULLUP);

	// Create PID instance for this motor instance
	_speedPID = new PID_v2(SPEED_KP, SPEED_KI, SPEED_KD, PID::Direct, PID::P_On::Measurement);
	_speedPID->SetMode(MANUAL);
	_speedPID->SetOutputLimits(MIN_DRIVE_VALUE, MAX_DRIVE_VALUE);	// Set PWM min and max
}

int mySign(int num)
{
	if (num < 0) {
		return -1;
	}
	if (num > 0) {
		return +1;
	}
	return 0;
}

void Motor::pulseHandlerLeft() {
	SideVars *sidePtr = &sides[LEFT];
	if (sidePtr->currentDir == 0) {
		sidePtr->distanceCount += sidePtr->prevDir;
	} else {
		sidePtr->distanceCount += sidePtr->currentDir;
	}
}
void Motor::pulseHandlerRight() {
	SideVars *sidePtr = &sides[RIGHT];
	if (sidePtr->currentDir == 0) {
		sidePtr->distanceCount += sidePtr->prevDir;
	} else {
		sidePtr->distanceCount += sidePtr->currentDir;
	}
}


/* drive(speed) sets the PWM output
 * Args: speed -100 - 0 - +100
 */
void Motor::drive(int speed) {
	unsigned long currentMicros = millis();
	unsigned long deltaMicros;
	SideVars *sidePtr = &sides[_sideInstance];
	if (currentMicros < _prevMicros) { // overflow
		deltaMicros = _prevMicros + currentMicros + currentMicros;
	} else {
		deltaMicros = currentMicros - _prevMicros;
	}
	_prevMicros = currentMicros;

	long deltaCount = _prevDistanceCount - sidePtr->distanceCount;

	_currentSpeed = deltaCount * 1000000 / deltaMicros;		// in distCounts/s
	_requiredSpeed = speed * SPEEDSCALER;
	_speedPID->Compute();

	// Eval driving direction
	byte newDir = mySign(_driveValue);
	if(sidePtr->currentDir != newDir ) {
		// Change of direction
		sidePtr->prevDir = sidePtr->currentDir;
	}
	sidePtr->currentDir = newDir;

	// Update PWM driver
	if (_driveValue > 0) {
		digitalWrite(_dirPin1, LOW);
		digitalWrite(_dirPin2, HIGH);
		analogWrite(_speedPin, _driveValue);
	}
	if (_driveValue < 0) {
		digitalWrite(_dirPin1, HIGH);
		digitalWrite(_dirPin2, LOW);
		analogWrite(speedPinL, abs(_driveValue));
	}
	if (_driveValue == 0) {
		digitalWrite(_dirPin1, LOW);
		digitalWrite(_dirPin2, LOW);
	}
}

long Motor::getDistance() {
	return(sides[_sideInstance].distanceCount);
}

double Motor::getCurrentSpeed() {
	return(_currentSpeed);
}


Motor::~Motor() {
	// TODO Auto-generated destructor stub
}

