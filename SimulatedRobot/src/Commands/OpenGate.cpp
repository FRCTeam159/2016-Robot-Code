/*
 * OpenGate.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: alpiner
 */

#include <Commands/OpenGate.h>
#include "Robot.h"
OpenGate::OpenGate() : Command("OpenGate")  {
	Requires(Robot::holder.get());
	std::cout << "new OpenGate"<< std::endl;
}

// Called just before this Command runs the first time
void OpenGate::Initialize() {
	Robot::holder->OpenGate();
	std::cout << "OpenGate started .."<<std::endl;
}
bool OpenGate::IsFinished() {
	bool done=Robot::holder->IsGateOpen();
	//std::cout << "OpenGate complete"<< std::endl;
	return done;
}
void OpenGate::End() {
	std::cout << "OpenGate End"<<std::endl;
}
