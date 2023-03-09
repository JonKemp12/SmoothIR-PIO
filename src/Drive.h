/*
 * Drive.h - Library for Driving Osoyoo Model 3.0 Robot Car motor.
 *  Includes PID speed control
 *
 *  Created on: 18 Mar 2022
 *      Author: jonkemp
 */


#ifndef DRIVE_H_
#define DRIVE_H_

#include "Arduino.h"
#include <PID_v1.h>

class Drive {
public:
	Drive();
private:
	PID speedPID;
};



#endif /* DRIVE_H_ */
