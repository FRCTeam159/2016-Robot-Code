/*
 * Shooter.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */
#include "Assignments.h"
#include <Subsystems/Shooter.h>

#define FWSPEED 300
#define FP 0.001
#define FI 0.00001
#define FD 0.0001
#define AP 0.2
#define AI 0.001
#define AD 0.3
#define FVMAX 1000
#define FVMIN -1000
#define AMIN 0
#define AMAX 70

Shooter::Shooter() : Subsystem("Shooter"),
	angleMotor(SHOOTER_ANGLE,true), leftMotor(SHOOTER_LEFT,true),rightMotor(SHOOTER_RIGHT,true)
{
	std::cout<<"New Shooter("<<SHOOTER_ANGLE<<","<<SHOOTER_LEFT<<","<<SHOOTER_RIGHT<<")"<<std::endl;
	max_angle=AMAX; // max elevation (degrees)
	min_angle=AMIN;

	angle=0;
	angleMotor.SetPID(GPMotor::POSITION, AP, AI, AD);
	angleMotor.Reset(); // clear IAccum
	angleMotor.SetDistancePerPulse(1.0); // 1 degree = 0.01745 radians
	angleMotor.SetInputRange(min_angle,max_angle);      // 0..70 degrees
	angleMotor.SetTolerance(1); // accept 1 degrees max error

	angleMotor.SetDistance(0);
	leftMotor.SetInputRange(FVMIN,FVMAX);
	rightMotor.SetInputRange(FVMIN,FVMAX);

	leftMotor.SetPID(GPMotor::SPEED, FP, FI, FD);
	leftMotor.SetDistancePerPulse(RPD(1)); // 1 degree = 0.01745 radians
	rightMotor.SetPID(GPMotor::SPEED,  FP, FI, FD);
	rightMotor.SetDistancePerPulse(RPD(1));

	flywheel_target=FWSPEED;
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
	leftMotor.SetDebug(0);
	rightMotor.SetDebug(0);
}

// Initialize
void Shooter::Init(){
	angleMotor.ClearIaccum();
	angleMotor.Enable();
	leftMotor.SetVelocity(0);
	rightMotor.SetVelocity(0);

	leftMotor.Enable();
	rightMotor.Enable();
	//rightMotor.SetDebug(2);
	//leftMotor.SetDebug(2);
}

void Shooter::Disable(){
	angleMotor.Reset();
	angleMotor.Disable();
	leftMotor.Disable();
	rightMotor.Disable();

	angleMotor.DisablePID();
	leftMotor.DisablePID();
	rightMotor.DisablePID();

	angle=0;
}
// Set the shooter angle
void Shooter::SetTargetAngle(double a){
	a=a>max_angle?max_angle:a;
	a=a<min_angle?min_angle:a;
	angle=a;
	angleMotor.SetDistance(angle);
	angleMotor.EnablePID();
}

// Set the shooter angle
void Shooter::SetTargetSpeed(double a){
	flywheel_target=a;
}

bool Shooter::IsAtAngle(){
	bool ontarget= angleMotor.OnTarget();
	return ontarget;
}
double Shooter::GetTargetAngle(){
	return angle;
}
double Shooter::GetTargetSpeed(){
	return flywheel_target;
}
double Shooter::GetSpeed(){
	double ave_speed=leftMotor.GetVelocity()+rightMotor.GetVelocity();
	return ave_speed/2;
}

void Shooter::EnableFlywheels(){
	leftMotor.SetVelocity(flywheel_target);
	rightMotor.SetVelocity(flywheel_target);
	leftMotor.EnablePID();
	rightMotor.EnablePID();
}
void Shooter::DisableFlywheels(){
	leftMotor.SetVelocity(0);
	rightMotor.SetVelocity(0);
	leftMotor.DisablePID();
	rightMotor.DisablePID();
}

bool Shooter::IsAtSpeed(){
	bool ontarget= leftMotor.OnTarget() && rightMotor.OnTarget();
	return ontarget;
}

