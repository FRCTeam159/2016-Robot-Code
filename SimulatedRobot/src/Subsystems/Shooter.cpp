/*
 * Shooter.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */
#include "Assignments.h"
#include <Subsystems/Shooter.h>
#include <Commands/InitShooter.h>

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
#define MAX_SPEED_ERROR 10
#define MAX_ANGLE_ERROR 1
#define SETZEROSPEED -0.6

Shooter::Shooter() : Subsystem("Shooter"),
	angleMotor(SHOOTER_ANGLE,false),
	leftMotor(SHOOTER_LEFT,true),
	rightMotor(SHOOTER_RIGHT,true),
	accel(SHOOTER_PITCH),
	lowerLimit(SHOOTER_MIN)

{
	std::cout<<"New Shooter("<<SHOOTER_ANGLE<<","<<SHOOTER_LEFT<<","<<SHOOTER_RIGHT<<")"<<std::endl;
	max_angle=AMAX; // max elevation (degrees)
	min_angle=AMIN;

	leftMotor.SetInputRange(FVMIN,FVMAX);
	rightMotor.SetInputRange(FVMIN,FVMAX);

	leftMotor.SetPID(GPMotor::SPEED, FP, FI, FD);
	leftMotor.SetDistancePerPulse(RPD(1)); // 1 degree = 0.01745 radians
	leftMotor.SetTolerance(MAX_SPEED_ERROR);

	rightMotor.SetPID(GPMotor::SPEED,  FP, FI, FD);
	rightMotor.SetDistancePerPulse(RPD(1));
	rightMotor.SetTolerance(MAX_SPEED_ERROR);

	flywheel_target=FWSPEED;
	flywheel_speed=0;

	angle=0;
	angleMotor.SetPID(AP, AI, AD,this);
	//angleMotor.Reset(); // clear IAccum
	//angleMotor.SetDistancePerPulse(1.0); // 1 degree = 0.01745 radians
	//angleMotor.SetDistance(0);
	//angleMotor.SetInputRange(min_angle,max_angle);      // 0..70 degrees
	angleMotor.SetTolerance(MAX_ANGLE_ERROR);
	//angleMotor.SetToleranceBuffer(2);
	initialized=false;
	Log();
}

double Shooter::PIDGet() {
	return accel.GetAngle();
}

void Shooter::InitDefaultCommand() {
	SetDefaultCommand(new InitShooter());
}

void Shooter::AutonomousInit(){
	std::cout << "Shooter::AutonomousInit"<<std::endl;
	Init();
}
void Shooter::TeleopInit(){
	std::cout << "Shooter::TeleopInit"<<std::endl;
	Init();
}
void Shooter::DisabledInit(){
	std::cout << "Shooter::DisabledInit"<<std::endl;
	Disable();
	angleMotor.SetDebug(0);
	leftMotor.SetDebug(0);
	rightMotor.SetDebug(0);
}

void Shooter::LogSpeed(double d) {
	SmartDashboard::PutNumber("Shooter FW Speed", d);
}
void Shooter::LogAngle(double d) {
	SmartDashboard::PutNumber("Shooter Angle", d);
}

void Shooter::Log() {
	LogAngle(GetAngle());
	LogSpeed(GetSpeed());
}


// Initialize
void Shooter::Init(){
	//angleMotor.SetDebug(2);
	initialized=false;
	angleMotor.SetTolerance(MAX_ANGLE_ERROR);
	angleMotor.ClearIaccum();
	angleMotor.Enable();
	leftMotor.SetVelocity(0);
	rightMotor.SetVelocity(0);

	leftMotor.Enable();
	rightMotor.Enable();

	Log();
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
	initialized=false;

	Log();
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
	GetAngle();
	return ontarget;
}
bool Shooter::IsAtSpeed(){
	bool ontarget= leftMotor.OnTarget() && rightMotor.OnTarget();
	GetSpeed();
	return ontarget;
}

double Shooter::GetTargetAngle(){
	return angle;
}
double Shooter::GetTargetSpeed(){
	return flywheel_target;
}
double Shooter::GetSpeed(){
	double ave_speed=(leftMotor.GetVelocity()+rightMotor.GetVelocity());
	LogSpeed(ave_speed);
	return ave_speed;
}
double Shooter::GetAngle(){
	double d=accel.GetAngle();
	LogAngle(d);
	return d;
}

void Shooter::EnableFlywheels(){
	leftMotor.SetVelocity(flywheel_target);
	rightMotor.SetVelocity(flywheel_target);
	leftMotor.EnablePID();
	rightMotor.EnablePID();
	Log();
}
void Shooter::DisableFlywheels(){
	leftMotor.SetVelocity(0);
	rightMotor.SetVelocity(0);
	leftMotor.DisablePID();
	rightMotor.DisablePID();
	Log();
}

void Shooter::SetInitialized() {
	initialized=true;
	angleMotor.Set(0);
}
void Shooter::Initialize() {
	if(!AtLowerLimit())
		GoToLowerLimitSwitch();
	else
		initialized=true;
}
bool Shooter::IsInitialized() {
	if(initialized)
		return true;
	else
		return AtLowerLimit();
}

void Shooter::GoToLowerLimitSwitch() {
	angleMotor.SetVoltage(SETZEROSPEED);
}

bool Shooter::AtLowerLimit() {
	return lowerLimit.Get();
}
