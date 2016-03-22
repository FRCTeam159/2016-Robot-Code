/*
 * Loader.cpp
 *
 *  Created on: Mar 21, 2016
 *      Author: alpiner
 */
#include "Assignments.h"
#include <Subsystems/Loader.h>

Loader::Loader() : Subsystem("Loader"),
  angleMotor(LOADER_ANGLE,false), rollerMotor(LOADER_ROLLERS,false),angleGyro(LOADER_PITCH)
{
}

void Loader::Log() {
}

void Loader::Disable(){
	angleMotor.Reset();
	angleMotor.Disable();
	rollerMotor.Disable();

	angle=0;
	Log();
}

// Initialize
void Loader::Init(){
}

void Loader::AutonomousInit(){
	Init();
}
void Loader::TeleopInit(){
	Init();
}
void Loader::DisabledInit(){

}

void Loader::SetTargetAngle(double a){
	a=a>max_angle?max_angle:a;
	a=a<min_angle?min_angle:a;
	angle=a;
	angleMotor.SetDistance(angle);
	angleMotor.EnablePID();
}
bool Loader::IsAtAngle(){
	return angleMotor.OnTarget();
}

double Loader::GetTargetAngle(){
	return angle;
}
