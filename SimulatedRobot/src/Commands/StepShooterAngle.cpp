/*
 * StepShooterAngle.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Commands/StepShooterAngle.h>
#include "Robot.h"

StepShooterAngle::StepShooterAngle(double a) : Command("StepShooterAngle") {
	//Requires((Subsystem*)Robot::shooter.get());
	std::cout << "new StepShooterAngle("<<a<<")"<< std::endl;
	direction=a;
}
// Called just before this Command runs the first time
void StepShooterAngle::Initialize() {
	double current=Robot::shooter->GetTargetAngle();
	double max=Robot::shooter->GetMaxAngle();
	double min=Robot::shooter->GetMinAngle();

	double target=current+direction;
	target=target>=max?max:target;
	target=target<=min?min:target;
	std::cout << "Changing Shooter Angle - current:"<< current <<" new:"<<target<<std::endl;
	Robot::shooter->SetTargetAngle(target);
}

// Called repeatedly when this Command is scheduled to run
void StepShooterAngle::Execute() {
}
// Make this return true when this Command no longer needs to run execute()
bool StepShooterAngle::IsFinished() {
	bool ontarget=Robot::shooter->IsAtAngle();
	if(ontarget)
		std::cout << "Shooter On Target:"<<std::endl;

	return ontarget;
}
// Called once after isFinished returns true
void StepShooterAngle::End() {
	//Robot::shooter->DisablePID();
}
// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void StepShooterAngle::Interrupted() {
	End();
}
