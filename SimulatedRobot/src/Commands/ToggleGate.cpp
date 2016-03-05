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
	current_state=UNDEFINED;
	target_state=CLOSED;
}
// Called just before this Command runs the first time
void ToggleGate::Initialize() {
	if (Robot::holder->IsGateOpen()){  //If the open gate button is pressed, close the gate.
		current_state=OPEN;
		target_state=CLOSED;
		std::cout << "ToggleGate gate is currently Open: Closing ..."<< std::endl;
		Robot::holder->CloseGate();
	}
	else if (Robot::holder->IsGateClosed()){//Otherwise, keep claws open.
		current_state=CLOSED;
		target_state=OPEN;
		std::cout << "ToggleGate gate is currently Closed: Opening ..."<< std::endl;
		Robot::holder->OpenGate();
	}
	else{
		current_state=UNDEFINED;
		current_state=CLOSED;
		Robot::holder->CloseGate();
		std::cout << "ToggleGate gate is not at either limit: Closing.."<< std::endl;
	}
}

// Called repeatedly when this Command is scheduled to run
void ToggleGate::Execute() {
	//Robot::holder->IsBallPresent();
}
// Make this return true when this Command no longer needs to run execute()
bool ToggleGate::IsFinished() {
	bool ontarget=false;
	switch(target_state){
	case CLOSED:
		ontarget=Robot::holder->IsGateClosed();
		break;
	case OPEN:
		ontarget=Robot::holder->IsGateOpen();
		break;
	}
	if(ontarget){
		std::cout << "ToggleGate on target"<< std::endl;
	}
	return ontarget;
}
// Called once after isFinished returns true
void ToggleGate::End() {
	//Robot::holder->CloseGate();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ToggleGate::Interrupted() {
	End();
}
