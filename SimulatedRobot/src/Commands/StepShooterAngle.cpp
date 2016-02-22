/*
 * StepShooterAngle.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Commands/StepShooterAngle.h>
#include "Robot.h"

StepShooterAngle::StepShooterAngle(double a) : Command("StepShooterAngle") {
	Requires(Robot::shooter.get());
}
// Called just before this Command runs the first time
void StepShooterAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void StepShooterAngle::Execute() {
	//Robot::drivetrain->Drive(Robot::oi->GetJoystick());
}
// Make this return true when this Command no longer needs to run execute()
bool StepShooterAngle::IsFinished() {
	return false;
}
// Called once after isFinished returns true
void StepShooterAngle::End() {
	//Robot::drivetrain->Drive(0, 0);
}
// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void StepShooterAngle::Interrupted() {
	End();
}
