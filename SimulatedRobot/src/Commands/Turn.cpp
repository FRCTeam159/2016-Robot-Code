/*
 * Turn.cpp
 *
 *  Created on: Mar 13, 2016
 *      Author: dean
 */

#include <Commands/Turn.h>
#include "Robot.h"

#define MAX_HEADING_ERROR 1 // degrees
#define TURN_TIMEOUT 2 // degrees

#define AP 0.2
#define AI 0.001
#define AD 0.01

//#define DEBUG_COMMAND

Turn::Turn(double a)  : Command("Turn"), pid(AP,AI,AD,this,this,SIMRATE)
{
	target=a;
	Requires(Robot::drivetrain.get());
	std::cout << "new Turn("<<target<<")"<< std::endl;
}

void Turn::Initialize() {
	SetTimeout(TURN_TIMEOUT);
	std::cout << "Turn Started .."<<std::endl;
	pid.Reset();
	pid.SetAbsoluteTolerance(MAX_HEADING_ERROR);
	pid.SetSetpoint(target);
	pid.Enable();

}
bool Turn::IsFinished() {
	if(IsTimedOut()){
		std::cout << "Turn Error:  Timeout expired"<<std::endl;
		return true;
	}
	return pid.OnTarget();
}

void Turn::End() {
	double h=Robot::drivetrain->GetHeading();
	std::cout << TimeSinceInitialized()<< "  Turn End("<<h<<")"<<std::endl;
	pid.Disable();
	Robot::drivetrain->EndTravel();
}

double Turn::PIDGet()
{
	double d=Robot::drivetrain->GetHeading();
#ifdef DEBUG_COMMAND
	std::cout << "Turn::PIDGet("<<d<<")"<<std::endl;
#endif
	return d;
}
void Turn::PIDWrite(float d)
{
#ifdef DEBUG_COMMAND
	std::cout << "Turn::PIDWrite("<<d<<")"<<std::endl;
#endif
	Robot::drivetrain->Turn(-d);
}
