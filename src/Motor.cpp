/*
 * Motor.cpp
 * All the functions to control a motor's movements
 *
 *  Created on: 20 Mar 2022
 *      Author: jonkemp
 */

#include "Motor.h"

Motor::Motor(SIDE side) {
	if (side == LEFT) {
		_speedPin = speedPinL;
		_dirPin1 = LeftDirectPin1;
		_dirPin2 = LeftDirectPin2;
		_pulsePin = 2;
	} else {
		_speedPin = speedPinR;
		_dirPin1 = RightDirectPin1;
		_dirPin2 = RightDirectPin2;
		_pulsePin = 3;
	}
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

void Motor::_pulseHandler() {
	if (_currentDir == 0) {
		distanceCount += _prevDir;
	} else {
		distanceCount += _currentDir;
	}
}

/* Typically called from the setup() to initialise the motor IO
 */
void Motor::setup() {
	pinMode(_dirPin1, OUTPUT);
	pinMode(_dirPin2, OUTPUT);
	pinMode(_speedPin, OUTPUT);

	// Attach motor encoder pulse pin to handler to maintain distance count
	pinMode(_pulsePin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(_pulsePin),_pulseHandler, CHANGE);

	_speedPID(&_currentSpeed, &_requiredSpeed, &_driveValue, SPEED_KP, SPEED_KI, SPEED_KD, DIRECT);
	_speedPID.SetMode(MANUAL);
	_speedPID.SetOutputLimits(MIN_DRIVE_VALUE, MAX_DRIVE_VALUE);	// Set PWM min and max
}

/* drive(speed) sets the PWM output
 * Args: speed -100 - 0 - +100
 */
void Motor::drive(int speed) {
	unsigned long currentMicros = millis();
	unsigned long deltaMicros;
	if (currentMicros < _prevMicros) { // overflow
		deltaMicros = _prevMicros + currentMicros + currentMicros;
	} else {
		deltaMicros = currentMicros - _prevMicros;
	}
	_prevMicros = currentMicros;

	long deltaCount = _prevDistanceCount - distanceCount;

	_currentSpeed = deltaCount * 1000000 / deltaMicros;		// in distCounts/s
	_requiredSpeed = speed * SPEEDSCALER;
	_speedPID.Compute();

	// Eval driving direction
	byte newDir = mySign(_driveValue);
	if(_currentDir != newDir ) {
		// Change of direction
		_prevDir = _currentDir;
	}
	_currentDir = newDir;

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
	return(distanceCount);
}

double Motor::getCurrentSpeed() {
	return(_currentSpeed);
}


Motor::~Motor() {
	// TODO Auto-generated destructor stub
}

