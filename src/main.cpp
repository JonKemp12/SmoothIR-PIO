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
#define speedPinR 5    //  RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1  7    //Right Motor direction pin 1 to MODEL-X IN1
#define RightDirectPin2  8    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6    // Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1  9    //Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2  10   //Left Motor direction pin 1 to MODEL-X IN4

#define motorBiasL	2.2		// Left motor balance multiplier
#define motorBiasR	2.2		// Left motor balance multiplier

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
} Drive_Num = DEF;

Motor leftMotor = Motor(LEFT);
Motor rightMotor = Motor(RIGHT);

bool stopFlag = true; //set stop flag
bool JogFlag = false;
/***************motor control***************/

/* Drive
 * Read global variable for current commanded speed L & R
 * Accelerate or decelerate until current speed = commanded speed.
 * Speed is -100 - + 100
 */
unsigned long previousMillis = 0;       // will store last time LED was updated
const long interval = 10; 				// ms interval between speed change increments 0 to 100 in 1 sec
unsigned int tickCounter = 0;			// Counter of ticks - for actions that are multiples of ticks. (mod(tickCounter))
int cmdSpeedL = 0;
int cmdSpeedR = 0;
int curSpeedL = 0;
int curSpeedR = 0;

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


void updateLeftDrive()
{
	curSpeedL -= mySign(curSpeedL - cmdSpeedL);

	int balancedSpeedValue = abs(curSpeedL * motorBiasL);


	if (curSpeedL > 0) {
		digitalWrite(LeftDirectPin1, LOW);
		digitalWrite(LeftDirectPin2, HIGH);
		analogWrite(speedPinL, balancedSpeedValue);
	}
	if (curSpeedL < 0) {
		digitalWrite(LeftDirectPin1, HIGH);
		digitalWrite(LeftDirectPin2, LOW);
		analogWrite(speedPinL, balancedSpeedValue);
	}
	if (curSpeedL == 0) {
		digitalWrite(LeftDirectPin1, LOW);
		digitalWrite(LeftDirectPin2, LOW);
	}
}

void updateRightDrive()
{
	curSpeedR -= mySign(curSpeedR - cmdSpeedR);

	int balancedSpeedValue = abs(curSpeedR * motorBiasR);


	if (curSpeedR > 0) {
		digitalWrite(RightDirectPin1, LOW);
		digitalWrite(RightDirectPin2, HIGH);
		analogWrite(speedPinR, balancedSpeedValue);
	}
	if (curSpeedR < 0) {
		digitalWrite(RightDirectPin1, HIGH);
		digitalWrite(RightDirectPin2, LOW);
		analogWrite(speedPinR, balancedSpeedValue);
	}
	if (curSpeedR == 0) {
		digitalWrite(RightDirectPin1, LOW);
		digitalWrite(RightDirectPin2, LOW);
	}
}

/*
 * Slowly reduce cmdSpeeds L & R
 */
void bleedSpeeds()
{
	if ((tickCounter % 5) == 0) {
		cmdSpeedL = cmdSpeedL - mySign(cmdSpeedL);
		cmdSpeedR = cmdSpeedR - mySign(cmdSpeedR);
	}
}

/*
 * Slowly reduce cmdSpeed difference cause by turn
 */
void turnSelfCentre() {
	if ((tickCounter % 3) == 0) {
		int diffLR = mySign(cmdSpeedL-cmdSpeedR);	// L>R = +1, R>L = -1, R==L = 0
		cmdSpeedL = cmdSpeedL - diffLR;
		cmdSpeedR = cmdSpeedR + diffLR;
	}
}

void printReport() {
	if ((tickCounter % 10) == 0) {
		Serial.print(tickCounter);
		Serial.print(" *** cmdL:");
		Serial.print(cmdSpeedL);
		Serial.print(" - cmdR:");
		Serial.print(cmdSpeedR);
		Serial.print(" * curL:");
		Serial.print(curSpeedL);
		Serial.print(" - curR:");
		Serial.print(curSpeedR);
		Serial.print(" ***");

		Serial.println("");
	}
}

void doDriveTick()
{
	unsigned long currentMillis = millis();

	if (currentMillis - previousMillis > interval) {
		tickCounter++;
		updateLeftDrive();
		updateRightDrive();
		bleedSpeeds();
		turnSelfCentre();
		// printReport();
		previousMillis = currentMillis;
	}
}


int maxChange(int change, int curValue) {
	int newValue = curValue + change;
	if (newValue > 100) {
		return 100;
	} else if (newValue < -100) {
		return -100;
	}
	return newValue;
}

void changeSpeeds(int leftChange, int rightChange) {
	cmdSpeedL = maxChange(leftChange, cmdSpeedL);
	cmdSpeedR = maxChange(rightChange, cmdSpeedR);

}
void doIRTick() {
	if (IR.decode(&IRresults)) {
		switch (IRresults.value) {
			case IR_ADVANCE:
				changeSpeeds(+40, +40);
				break;
			case IR_RIGHT:
				changeSpeeds(+20, -20);
				break;
			case IR_LEFT:
				changeSpeeds(-20, +20);
				break;
			case IR_BACK:
				changeSpeeds(-30, -30);
				break;
			case IR_STOP:
				cmdSpeedL = 0;
				cmdSpeedR = 0;
				break;
			default:
				break;
		}

		IRresults.value = 0;
		IR.resume();
	}
}


void setup() {
	pinMode(RightDirectPin1, OUTPUT);
	pinMode(RightDirectPin2, OUTPUT);
	pinMode(speedPinL, OUTPUT);
	pinMode(LeftDirectPin1, OUTPUT);
	pinMode(LeftDirectPin2, OUTPUT);
	pinMode(speedPinR, OUTPUT);
	cmdSpeedL = 0;
	cmdSpeedR = 0;

	pinMode(IR_PIN, INPUT);
	digitalWrite(IR_PIN, HIGH);
	IR.enableIRIn();

	Serial.begin(9600);
}

void loop() {
	doIRTick();
	doDriveTick();
}
