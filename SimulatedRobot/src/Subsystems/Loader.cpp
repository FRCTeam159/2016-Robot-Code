/*
 * Loader.cpp
 *
 *  Created on: Mar 21, 2016
 *      Author: alpiner
 */
#include <Commands/InitLoader.h>
#include "Assignments.h"
#include <Subsystems/Loader.h>

#define ROLLER_SPEED 0.5
#define SETZEROSPEED -0.6
#define ROLLERMOTORSPEED 1
#define MED_ANGLE 7.0
#define HIGH_ANGLE 17.5
#define LOW_ANGLE 0.0
#define MAX_ANGLE 50

#define MAX_ANGLE_ERROR 1
#define P 0.1
#define I 0.001
#define D 0.5

Loader::Loader() : Subsystem("Loader"),
  liftMotor(LOADER_ANGLE,false),
  rollerMotor(LOADER_ROLLERS,false),
  accel(LOADER_PITCH),
  lowerLimit(LOADER_MIN)
{
	liftMotor.SetPID(P,I,D,this);
	liftMotor.SetTolerance(MAX_ANGLE_ERROR);
	liftMotor.Disable();
	initialized=false;
}

void Loader::InitDefaultCommand() {
	SetDefaultCommand(new InitLoader());
}

void Loader::Log() {
}

void Loader::Disable(){
	liftMotor.Reset();
	liftMotor.Disable();
	rollerMotor.Disable();
	angle=0;
	liftMotor.SetDebug(0);
	initialized=false;
	Log();
}

// Initialize
void Loader::Init(){
	initialized=false;
}

void Loader::AutonomousInit(){
	std::cout << "Loader::AutonomousInit"<<std::endl;
	Init();
}
void Loader::TeleopInit(){
	std::cout << "Loader::TeleopInit"<<std::endl;
	Init();
}
void Loader::DisabledInit(){
	std::cout << "Loader::DisabledInit"<<std::endl;
	Disable();
}

void Loader::SetTargetAngle(double a){
	a=a>max_angle?max_angle:a;
	a=a<min_angle?min_angle:a;
	angle=a;
	liftMotor.SetSetpoint(angle);
	liftMotor.Enable();
}
bool Loader::AtAngle(){
	return liftMotor.OnTarget();
}

double Loader::GetTargetAngle(){
	return angle;
}

void Loader::TurnRollerOn(bool b) {
	if(b){
		rollerMotor.SetVoltage(ROLLER_SPEED);
		rollers_on=true;
	}
	else{
		rollerMotor.SetVoltage(0);
		rollers_on=false;
	}
}

bool Loader::AreRollersOn() {
	return rollers_on;
}

double Loader::PIDGet() {
	return -accel.GetAngle();
}

void Loader::GoToLowerLimitSwitch() {
	liftMotor.SetVoltage(SETZEROSPEED);
}

void Loader::SetLow() {
	liftMotor.SetSetpoint(LOW_ANGLE);
}

void Loader::SetMed() {
	liftMotor.SetSetpoint(MED_ANGLE);
}

void Loader::SetHigh() {
	liftMotor.SetSetpoint(HIGH_ANGLE);
}

void Loader::SetMax() {
	liftMotor.SetSetpoint(MAX_ANGLE);
}

bool Loader::AtLowerLimit() {
	return lowerLimit.Get();
}

void Loader::SetInitialized() {
	initialized=true;
	liftMotor.Set(0);
}
void Loader::Initialize() {
	if(!AtLowerLimit())
		GoToLowerLimitSwitch();
	else
		initialized=true;
}
bool Loader::IsInitialized() {
	return initialized;
//	if(initialized)
//		return true;
//	else
//		return AtLowerLimit();
}
