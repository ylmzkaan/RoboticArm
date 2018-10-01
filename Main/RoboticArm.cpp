#include "Encoder.h"
#include "PID_v1.h" 
#include "DualVNH5019MotorShield.h"
#include "RoboticArm.h"

extern double ref1, input1, output1, p, i, d;
extern double ref2, input2, output2, p2, i2, d2;
extern double ref3, input3, output3, p3, i3, d3;
extern double ref4, input4, output4, p4, i4, d4;

extern DualVNH5019MotorShield motorDriver;

extern Encoder M1enc;
extern Encoder M2enc;
extern Encoder M3enc;
extern Encoder M4enc;

extern PID PID1;
extern PID PID2;
extern PID PID3;
extern PID PID4;

extern int rampDuration;

extern double finalAngularPositions[4];

double* currentTargetAngularPositions = new double[4];

void motorControl(int motor, double targetAngularPosition)
{
	switch (motor)
	{
	case 1:
		// Actual position of encoder1 in degrees. 
		// 2248.86 is 48*47 (encoder's total pulses in one revolution)*(reduction ratio)
		input1 = M1enc.read() / 2248.86 * 360;
		ref1 = targetAngularPosition; // In degrees
		// Set PID gains to p = 40, i = 20, d = 0
		PID1.SetTunings(40, 20, 0);
		// Compute PID output using input1, ref1 and pid gains
		PID1.Compute();
		// Set motor1's voltage to PID's output
		motorDriver.setM1Speed(output1);
		break;

	case 2:
		// Actual position of encoder2 in degrees. 4741.44 is 48*99 (encoder's total pulses in one revolution)*(reduction ratio). 
		// Additional 1/3 is for the reduction due to bevel gears.
		input2 = M2enc.read() / 4741.44 * 360 / 3;
		ref2 = targetAngularPosition;
		PID3.SetTunings(100, 50, 0);
		PID2.Compute();
		motorDriver.setM2Speed(output2);
		break;

	case 3:
		// Actual position of encoder3 in degrees. 4741.44 is 48*99 (encoder's total pulses in one revolution)*(reduction ratio). 
		// Additional 1/2 is for the reduction due to bevel gears.
		input3 = M3enc.read() / 4741.44 * 360 / 2;
		ref3 = targetAngularPosition;
		PID3.SetTunings(600, 300, 0);
		PID3.Compute();
		motorDriver.setM3Speed(output3);
		break;

	case 4:
		input4 = M4enc.read() / 2248.86 * 360 / 2;
		ref4 = targetAngularPosition;
		PID4.SetTunings(80, 20, 0);
		PID4.Compute();
		motorDriver.setM4Speed(output4);
		break;
	}
}

// When the emergency pushbutton is pushed, the pin that the pushbutton is connected to becomes low. 
// If it is low, all motors stop.
void emerStop()
{
	if (digitalRead(53) == LOW) {
		motorDriver.setSpeeds(0, 0, 0, 0);
		Serial.println("Emergency stop");
		while (true); // This is not elegant!
	}
}

void currentProtection()
{
	// Maximum drawable current
	const short currentLimit = 5500;

	if (motorDriver.getM1CurrentMilliamps() > currentLimit 
	    || motorDriver.getM2CurrentMilliamps() > currentLimit 
	    || motorDriver.getM3CurrentMilliamps() > currentLimit 
	    || motorDriver.getM4CurrentMilliamps() > currentLimit)
	{
		Serial.print("STOPPED DUE TO HIGH CURRENT");
		motorDriver.setSpeeds(0, 0, 0, 0);
		while (true);
	}
}

void stopIfFault()
{
	if (motorDriver.getM1Fault())
	{
		Serial.println("M1 fault");
		while (true);
	}
	else if (motorDriver.getM2Fault())
	{
		Serial.println("M2 fault");
		while (true);
	}
	else if (motorDriver.getM3Fault())
	{
		Serial.println("M3 fault");
		while (true);
	}
	else if (motorDriver.getM4Fault())
	{
		Serial.println("M4 fault");
		while (true);
	}
}

double* calcCurrentTargetAngularPositions(double* finalAngularPositions)
{
	if (millis() < rampDuration)
	{
		// Time passed since the beginning of the code
		double currentTime = millis();

		// Calculate at which position should the motor shafts be at the present time
		currentTargetAngularPositions[0] = currentTime * finalAngularPositions[0] / rampDuration; // Degrees
		currentTargetAngularPositions[1] = currentTime * finalAngularPositions[1] / rampDuration;
		currentTargetAngularPositions[2] = currentTime * finalAngularPositions[2] / rampDuration;
		currentTargetAngularPositions[3] = currentTime * finalAngularPositions[3] / rampDuration;
		return currentTargetAngularPositions;
	}
	else
	{
		// If the ramp signal's duration is reached, keep motors' shafts at target angular positions
		return finalAngularPositions;
	}
}

void driveMotorsToCurrentTargets(double* angularPositions)
{
	motorControl(1, angularPositions[0]);
	motorControl(2, angularPositions[1]);
	motorControl(3, angularPositions[2]);
	motorControl(4, angularPositions[3]);
}

void printData()
{
	Serial.print("Position1: ");
	Serial.print(input1); // current position of motor 1
	Serial.print(" Position2: ");
	Serial.print(input2);
	Serial.print(" Position3: ");
	Serial.print(input3);
	Serial.print(" Position4: ");
	Serial.print(input4);

	Serial.print(" Current1: ");
	Serial.print(motorDriver.getM1CurrentMilliamps()); // current drawn by motor 1
	Serial.print(" Current2: ");
	Serial.print(motorDriver.getM2CurrentMilliamps());
	Serial.print(" Current3: ");
	Serial.print(motorDriver.getM3CurrentMilliamps());
	Serial.print(" Current4: ");
	Serial.print(motorDriver.getM4CurrentMilliamps());

	Serial.print(" Speed1: ");
	Serial.print(output1); // pid output of motor 1
	Serial.print(" Speed2: ");
	Serial.print(output2);
	Serial.print(" Speed3: ");
	Serial.print(output3);
	Serial.print(" Speed4: ");
	Serial.println(output4);
}
	
