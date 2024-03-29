/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/
 * www.osoyoo.com IR remote control smart car
 * program tutorial https://osoyoo.com/2019/09/19/osoyoo-model-3-robot-learning-kit-lesson-2-ir-remote-controlled/
 *  Copyright John Yu
 */
#include <IRremote.h>
// #include <Math.h>
#include <Motor.h>

#define IR_PIN    12 //IR receiver Signal pin connect to Arduino pin D2
IRrecv IR(IR_PIN);  //   IRrecv object  IR get code from IR remoter
decode_results IRresults;

// Declare the two motors
static MotorCounter leftCounter;
static Motor leftMotor(LEFT, &leftCounter);
static MotorCounter rightCounter;
static Motor rightMotor(RIGHT, &rightCounter);


void pulseHandlerLeft() {
	MotorCounter *countPtr = &leftCounter;
	if (countPtr->currentDir == 0) {
		countPtr->distanceCount += countPtr->prevDir;
	} else {
		countPtr->distanceCount += countPtr->currentDir;
	}
}

void pulseHandlerRight() {
	MotorCounter *sidePtr = &rightCounter;
	if (sidePtr->currentDir == 0) {
		sidePtr->distanceCount += sidePtr->prevDir;
	} else {
		sidePtr->distanceCount += sidePtr->currentDir;
	}
}


#define IR_ADVANCE       0x00FF18E7       //code from IR controller "▲" button
#define IR_BACK          0x00FF4AB5       //code from IR controller "▼" button
#define IR_RIGHT         0x00FF5AA5       //code from IR controller ">" button
#define IR_LEFT          0x00FF10EF       //code from IR controller "<" button
#define IR_STOP          0x00FF38C7       //code from IR controller "OK" button
#define IR_turnsmallleft 0x00FFB04F       //code from IR controller "#" button

enum DN {
	GO_ADVANCE, //go forward
	GO_LEFT, //left turn
	GO_RIGHT, //right turn
	GO_BACK, //backward
	STOP_STOP,
	DEF
} Drive_Num;

void doIRTick() {
	if (IR.decode(&IRresults)) {
		switch (IRresults.value) {
			case IR_ADVANCE:
				// changeSpeeds(+40, +40);
				break;
			case IR_RIGHT:
				// changeSpeeds(+20, -20);
				break;
			case IR_LEFT:
				// changeSpeeds(-20, +20);
				break;
			case IR_BACK:
				// changeSpeeds(-30, -30);
				break;
			case IR_STOP:
				leftMotor.setDriveValue(0);
				rightMotor.setDriveValue(0);
				break;
			default:
				break;
		}

		IRresults.value = 0;
		IR.resume();
	}
}


void setup() 
{	
	attachInterrupt(digitalPinToInterrupt(leftCounter.pulsePin), pulseHandlerLeft, CHANGE);
	attachInterrupt(digitalPinToInterrupt(rightCounter.pulsePin), pulseHandlerRight, CHANGE);

	pinMode(IR_PIN, INPUT);
	digitalWrite(IR_PIN, HIGH);
	IR.enableIRIn();

	Serial.begin(9600);
}

void waitFor(unsigned long button)
{
	bool waitHere = true;

	while (waitHere)
	{
		if (IR.decode(&IRresults))
		{
			if (IRresults.value == button)
			{
				waitHere = false;
			}
			IRresults.value = 0;
			IR.resume();
		}
	}
}

// Update the motors with given speed values
void doMotorUpdate(int leftSpeed, int rightSpeed) {
	// leftMotor.drive(leftSpeed);
	rightMotor.drive(rightSpeed);
}


#define NUM_COUNTS 101
int times[NUM_COUNTS];
int leftCounts[NUM_COUNTS];
int rightCounts[NUM_COUNTS];
int leftDriveValues[NUM_COUNTS];
int rightDriveValues[NUM_COUNTS];

void loop() {
	// Wait for IR advance press
	Serial.println("Waiting for IR_ADVANCE");
	waitFor(IR_ADVANCE);

	// Drive and stop
	Serial.println("Running left & right at 100");

unsigned long startTime = millis();

	for (int i = 0; i < NUM_COUNTS; i++)
	{
		int loopStart = millis();
		int elapsedTime = 0;
		while(elapsedTime < 20) {
			doMotorUpdate(100, 100);
			elapsedTime = millis() - loopStart;
		}

		// Capture counts and times
		times[i] = millis() - startTime;
		leftCounts[i] = leftCounter.distanceCount;
		rightCounts[i] = rightCounter.distanceCount;
		leftDriveValues[i] = leftMotor.getDriveValue();
		rightDriveValues[i] = rightMotor.getDriveValue();
	}
	
	Serial.println("Running left & right at 0");
	leftMotor.setDriveValue(0);
	rightMotor.setDriveValue(0);

	// Wait for IR stop press
	Serial.println("Waiting for IR_STOP");
	waitFor(IR_STOP);

	// Print captured data
	Serial.println("Time,leftCount,rightCount,leftChange,rightChange");
	for (int i = 1; i < NUM_COUNTS; i++)
	{
		Serial.print(times[i]);
		Serial.print(",");
		Serial.print(leftCounts[i]);
		Serial.print(",");
		Serial.print(rightCounts[i]);
		Serial.print(",");
		Serial.print(leftDriveValues[i]);
		Serial.print(",");
		Serial.print(rightDriveValues[i]);
		Serial.println();
	}
	
	// doIRTick();
}
