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
	std::cout << "new ToggleMode()"<< std::endl;

}
// Called just before this Command runs the first time
void ToggleMode::Initialize() {
	bool old_state=OI::GetMode();
	if(old_state==OI::SHOOTING)
		OI::SetMode(OI::LOADING);
	else
		OI::SetMode(OI::SHOOTING);
	bool new_state=OI::GetMode();
	if(new_state==OI::SHOOTING)
		std::cout << "Changing Mode to Shooting"<<std::endl;
	else
		std::cout << "Changing Mode to Loading"<<std::endl;

	last_state=new_state;
}
