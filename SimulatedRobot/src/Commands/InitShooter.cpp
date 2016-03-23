/*
 * InitShooter.cpp
 *
 *  Created on: Mar 23, 2016
 *      Author: alpiner
 */

#include <Commands/InitShooter.h>
#include "Robot.h"

#define INIT_TIMEOUT 2

InitShooter::InitShooter() : Command("InitShooter")  {
	Requires(Robot::shooter.get());
	std::cout << "new InitShooter()"<< std::endl;
}

void InitShooter::Initialize() {
	SetTimeout(INIT_TIMEOUT);
	if(!Robot::shooter->IsInitialized()){
		std::cout << "InitShooter initializing Loader"<< std::endl;
		Robot::shooter->Initialize();
	}
}

bool InitShooter::IsFinished() {
	if(IsTimedOut()){
		std::cout << "InitShooter Error: Timeout expired"<<std::endl;
		return true;
	}
	return Robot::shooter->AtLowerLimit();
}

void InitShooter::End() {
	if(!Robot::shooter->IsInitialized()){
		std::cout << "InitShooter::End()"<< std::endl;
		Robot::shooter->SetInitialized();
	}
}
