/*
 * ToggleMode.cpp
 *
 *  Created on: Mar 21, 2016
 *      Author: alpiner
 */

#include <Commands/ToggleMode.h>
#include "Robot.h"
#include "OI.h"

ToggleMode::ToggleMode()  : Command("ToggleMode"){
	//Requires(Robot::oi.get());
	last_state=OI::GetMode();
	std::cout << "new ToggleMode("<<last_state<<")"<< std::endl;

}
// Called just before this Command runs the first time
void ToggleMode::Initialize() {
	bool old_state=OI::GetMode();
	if(old_state==OI::SHOOTING)
		OI::SetMode(OI::LOADING);
	else
		OI::SetMode(OI::SHOOTING);
	bool new_state=OI::GetMode();
	std::cout << "Changing Mode - prev:"<< old_state <<" new:"<<new_state<<std::endl;
	last_state=new_state;
}
