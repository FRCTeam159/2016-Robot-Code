/*
 * Shooter.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */
#include "Assignments.h"
#include <Subsystems/Shooter.h>

#define FWSPEED 800

Shooter::Shooter() : Subsystem("Shooter"),
	angleMotor(SHOOTER_ANGLE,true), leftMotor(SHOOTER_LEFT,true),rightMotor(SHOOTER_RIGHT,true)
{
	std::cout<<"New Shooter("<<SHOOTER_ANGLE<<","<<SHOOTER_LEFT<<","<<SHOOTER_RIGHT<<")"<<std::endl;
	angle=0;
	leftMotor.SetPID(GPMotor::SPEED, 0.5, 0.001, 1);
	leftMotor.SetDistancePerPulse(RPD(1)); // 1 degree = 0.01745 radians
	rightMotor.SetPID(GPMotor::SPEED, 0.5, 0.001, 1);
	rightMotor.SetDistancePerPulse(RPD(1));
	max_angle=70; // max elevation (degrees)
	min_angle=0;
}

void Shooter::AutonomousInit(){
	Init();
}
void Shooter::TeleopInit(){
	Init();
}
void Shooter::DisabledInit(){
	Disable();
	angleMotor.SetDebug(0);
}

// Initialize
void Shooter::Init(){
	angleMotor.SetPID(GPMotor::POSITION, 0.5, 0.0, 1);
	angleMotor.Reset(); // clear IAccum
	angleMotor.SetDistancePerPulse(1.0); // 1 degree = 0.01745 radians
	angleMotor.SetInputRange(min_angle,max_angle);      // 0..70 degrees
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
	a=a>max_angle?max_angle:a;
	a=a<min_angle?min_angle:a;
	angle=a;
	angleMotor.SetDistance(angle);
}

bool Shooter::OnTarget(){
	return angleMotor.OnTarget();
}
double Shooter::GetTargetAngle(){
	return angle;
}
