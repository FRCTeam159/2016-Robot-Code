/*
 * OpenGate.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Commands/ToggleGate.h>
#include "Robot.h"

ToggleGate::ToggleGate() : Command("ToggleGate") {
	Requires(Robot::holder.get());
	std::cout << "new ToggleGate"<< std::endl;
}
// Called just before this Command runs the first time
void ToggleGate::Initialize() {
	if (Robot::holder->GateIsOpen())  //If the opengate button is pressed, close the gate.
		Robot::holder->CloseGate();
	else  //Otherwise, keep claws open.
		Robot::holder->OpenGate();
	std::cout << "ToggleGate Open="<< Robot::holder->GateIsOpen()<< std::endl;
}

// Called repeatedly when this Command is scheduled to run
void ToggleGate::Execute() {
}
// Make this return true when this Command no longer needs to run execute()
bool ToggleGate::IsFinished() {
	return false;
}
// Called once after isFinished returns true
void ToggleGate::End() {
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ToggleGate::Interrupted() {
	End();
}
