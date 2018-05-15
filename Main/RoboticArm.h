#pragma once

void motorControl(int motor, double targetAngularPosition);

void emerStop();

void currentProtection();

void stopIfFault();

double* calcCurrentTargetAngularPositions(double* finalAngularPosition);

void driveMotorsToCurrentTargets(double* angularPositions);

void printData();
