/*
 * OpenGate.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Commands/ToggleGate.h>
#include "Robot.h"

ToggleGate::ToggleGate() : Command("ToggleGate") {
	// TODO Auto-generated constructor stub
}
// Called just before this Command runs the first time
void ToggleGate::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ToggleGate::Execute() {
	//Robot::drivetrain->Drive(Robot::oi->GetJoystick());
}
// Make this return true when this Command no longer needs to run execute()
bool ToggleGate::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ToggleGate::End() {
	//Robot::drivetrain->Drive(0, 0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ToggleGate::Interrupted() {
	End();
}
