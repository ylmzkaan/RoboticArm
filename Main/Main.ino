#include "Encoder.h"
#include "PID_v1.h" 
#include "DualVNH5019MotorShield.h"
#include "RoboticArm.h"

/*
Declare pid variables for each motor
ref1 is the reference signal. Basically, it is the target angular position for motor shaft 1. Motor 1's shaft's
angular position will eventually be what ref1 is.
input1 is the actual signal. It is the actual angular position of motor shaft 1.
output1 is the pid output for motor 1. It has a range between -400 and +400 and is proportional to power supplied
to motor 1
*/
double ref1, input1, output1, p, i, d;
double ref2, input2, output2, p2, i2, d2;
double ref3, input3, output3, p3, i3, d3;
double ref4, input4, output4, p4, i4, d4;

/*
DualVNH5019MotorShield::DualVNH5019MotorShield(
unsigned char INA1, unsigned char INB1,unsigned char EN1DIAG1, unsigned char CS1, unsigned char PWM1,
unsigned char INA2,unsigned char INB2, unsigned char EN2DIAG2, unsigned char CS2, unsigned char PWM2,
unsigned char INA3, unsigned char INB3, unsigned char EN3DIAG3, unsigned char CS3, unsigned char PWM3,
unsigned char INA4,unsigned char INB4, unsigned char EN4DIAG4, unsigned char CS4, unsigned char PWM4)
*/
DualVNH5019MotorShield motorDriver = DualVNH5019MotorShield(22, 24, 23, A0, 5,
                                                            26, 28, 25, A1, 11,
                                                            30, 32, 27, A2, 7,
                                                            34, 36, 29, A3, 8);

/*
Encoder constructers for all motors.
Encoders that came built-in with Pololu DC motors have two channels namely A and B.
Below constructers receive the pin mapping of channels A and B as input parameters, respectively.
For M1enc, 2 is the Arduino digital pin for channel A and 3 is the Arduino digital pin for channel B .
*/
Encoder M1enc(2, 3);
Encoder M2enc(18, 19);
Encoder M3enc(20, 31);
Encoder M4enc(21, 33);

// PID constructers
PID PID1(&input1, &output1, &ref1, p, i, d, DIRECT);
PID PID2(&input2, &output2, &ref2, p2, i2, d2, DIRECT);
PID PID3(&input3, &output3, &ref3, p3, i3, d3, DIRECT);
PID PID4(&input4, &output4, &ref4, p4, i4, d4, DIRECT);

// Error for safety purposes
bool error = false;
  
/*
A ramp signal can be visualized as below.
Amplitude
|     /--------
|    /
|   /
|  /
________________ Time
The ramp signal rises and then settles. Time that a ramp signal takes to complete the rise is the rampDuration.
For all 4 motors, same rampDuration will be used. in milliseconds
*/
int rampDuration = 3000;

/*
For each of the 4 motors, different target angular positions will be assigned.
Shortly after the time elapsed since the execution of this function reaches rampDuration, motor shafts' angular
positions will be as specified in targetAngularPosition array.
*/
double finalAngularPositions[4] = {0, 0, 30, 0};

void setup() {
	Serial.begin(250000);

	PID1.SetMode(AUTOMATIC);
	// Limits the maximum power of motor1
	PID1.SetOutputLimits(-300, 300);
	// Sampling time for motor1
	PID1.SetSampleTime(1);

	PID2.SetMode(AUTOMATIC);
	PID2.SetOutputLimits(-250, 250);
	PID2.SetSampleTime(1);

	PID3.SetMode(AUTOMATIC);
	PID3.SetOutputLimits(-250, 250);
	PID3.SetSampleTime(1);

	PID4.SetMode(AUTOMATIC);
	PID4.SetOutputLimits(-150, 150);
	PID4.SetSampleTime(1);

	// Emergency pushbutton setup
	pinMode(53, INPUT);
	digitalWrite(53, HIGH);

	// Initializes the functions and global variables of vnh5019 library
	motorDriver.init();

	// Reset all motors' encoder readings to zero.
	M1enc.write(0);
	M2enc.write(0);
	M3enc.write(0);
	M4enc.write(0);
}

void loop() {
	// Check if emergency pushbutton is pushed
	emerStop();
	// Check if there is a fault with the motors
	stopIfFault();
	// Check if any of the motors exceed current limit
	currentProtection();
  
	// Update the target angular positions for motor shafts
	/*
	Target angular positions at a specific time. During a ramp signal, target angular position of a particular motor's
	shaft depends on time. Thus, target angular position varies on time. currentTargetAngularPosition is an array that
	holds the current target angular position at any time.
	*/
	double* currentTargetAngularPositions = calcCurrentTargetAngularPositions(finalAngularPositions);
 
	driveMotorsToCurrentTargets(currentTargetAngularPositions);
	printData();
}
