/*
 * Shooter.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Subsystems/Shooter.h>

#define FWSPEED 400

Shooter::Shooter(int m1, int m2, int m3) : Subsystem("Shooter"),
	angleMotor(m1,true), leftMotor(m2,true),rightMotor(m3,true)
{
	std::cout<<"New Shooter("<<m1<<","<<m2<<","<<m3<<")"<<std::endl;
	angle=0;

	leftMotor.SetPID(GPMotor::SPEED, 0.4, 0, 0);
	leftMotor.SetDistancePerPulse(RPD(1)); // 1 degree = 0.01745 radians
	rightMotor.SetPID(GPMotor::SPEED, 0.4, 0, 0);
	rightMotor.SetDistancePerPulse(RPD(1)); // 1 degree = 0.01745 radians
}

// Initialize
void Shooter::Init(){
	angleMotor.SetPID(GPMotor::POSITION, 1.5, 0.001, 4);
	angleMotor.Reset();
	angleMotor.SetDistancePerPulse(1.0); // 1 degree = 0.01745 radians
	angleMotor.SetInputRange(0,70);      // 0..70 degrees
	//angleMotor.SetToleranceBuffer(1);
	angleMotor.SetTolerance(0.5); // accept 0.5 degrees max error

	angleMotor.SetDistance(0);
	angleMotor.Enable();

	leftMotor.SetVelocity(0);
	rightMotor.SetVelocity(0);
}

void Shooter::Disable(){
	angleMotor.Reset();
}
// Shoot the ball
void Shooter::Shoot(bool t){
	double speed=t?FWSPEED:0;
	leftMotor.SetVelocity(speed);
	rightMotor.SetVelocity(speed);
}
// Set the shooter angle
void Shooter::SetTargetAngle(double a){
	angle=a;
	angleMotor.SetDistance(angle);
}

bool Shooter::OnTarget(){
	return angleMotor.OnTarget();
}
double Shooter::GetTargetAngle(){
	return angle;
}
