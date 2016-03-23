/*
 * StepLoaderAngle.cpp
 *
 *  Created on: Mar 21, 2016
 *      Author: alpiner
 */

#include <Commands/StepLoaderAngle.h>
#include "Robot.h"

#define STEP_TIMEOUT 1

StepLoaderAngle::StepLoaderAngle(double a) : Command("StepLoaderAngle")  {
	Requires(Robot::loader.get());
	std::cout << "new StepLoaderAngle("<<a<<")"<< std::endl;
	direction=a;
}

void StepLoaderAngle::Initialize() {
	SetTimeout(STEP_TIMEOUT);
	double current=Robot::loader->GetTargetAngle();
	double target=current+direction;
	Robot::loader->SetTargetAngle(target);
	double newangle=Robot::loader->GetTargetAngle();
	std::cout << "Changing Loader Angle - current:"<< current <<" new:"<<newangle<<std::endl;
}

// Make this return true when this Command no longer needs to run execute()
bool StepLoaderAngle::IsFinished() {
	if(IsTimedOut()){
		std::cout << "StepLoaderAngle Error:  Timeout expired"<<std::endl;
		return true;
	}
	bool ontarget=Robot::loader->AtAngle();
	if(ontarget)
		std::cout << "Loader At Angle:"<<std::endl;
	return ontarget;
}
void StepLoaderAngle::End() {
	//Robot::shooter->DisablePID();
}
