/*
 * Motor.cpp
 * All the functions to control a motor's movements
 *
 *  Created on: 20 Mar 2022
 *      Author: jonkemp
 */

#include "Motor.h"

Motor::Motor(MOTORS side, MotorCounter *countPtr) {
	_sideInstance = side;			// Identify which side instance this is.
	_countPtr = countPtr;
	_countPtr->distanceCount = 0;
	_countPtr->currentDir = 0;
	_countPtr->prevDir = 0;
	if (side == LEFT) {
		_speedPin = speedPinL;
		_dirPin1 = LeftDirectPin1;
		_dirPin2 = LeftDirectPin2;
		_countPtr->pulsePin = LeftPulsePin;
	} else {
		_speedPin = speedPinR;
		_dirPin1 = RightDirectPin1;
		_dirPin2 = RightDirectPin2;
		_countPtr->pulsePin = RightPulsePin;
	}

	// Initialise motor IO pinModes
	pinMode(_dirPin1, OUTPUT);
	pinMode(_dirPin2, OUTPUT);
	pinMode(_speedPin, OUTPUT);
	pinMode(_countPtr->pulsePin, INPUT_PULLUP);

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

	long deltaCount = _prevDistanceCount - _countPtr->distanceCount;

	_currentSpeed = deltaCount * 1000000 / deltaMicros;		// in distCounts/s
	_requiredSpeed = speed * SPEEDSCALER;
	// speedPID.Compute takes currentSpeed and reqiredSpeed and calculates driveValue
	_speedPID->Compute();

	setDriveValue((int)_driveValue);
}

void Motor::setDriveValue(int value) {
	// Ensure -255 - 255
	value = (value < -255) ? -255 : value;
	value = (value >  255) ?  255 : value;
	
	_driveValue = (double)value;

	// Eval driving direction
	byte newDir = mySign(_driveValue);
	if(_countPtr->currentDir != newDir ) {
		// Change of direction
		_countPtr->prevDir = _countPtr->currentDir;
	}
	_countPtr->currentDir = newDir;

	// Update PWM driver
	if (value > 0) {
		digitalWrite(_dirPin1, LOW);
		digitalWrite(_dirPin2, HIGH);
		analogWrite(_speedPin, value);
	}
	if (value < 0) {
		digitalWrite(_dirPin1, HIGH);
		digitalWrite(_dirPin2, LOW);
		analogWrite(speedPinL, abs(value));
	}
	if (value == 0) {
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

