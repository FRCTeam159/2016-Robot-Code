/*
 * InitShooter.cpp
 *
 *  Created on: Mar 23, 2016
 *      Author: alpiner
 */

#include <Commands/InitShooter.h>
#include "Robot.h"

InitShooter::InitShooter() : Command("InitShooter")  {
	Requires(Robot::shooter.get());
	std::cout << "new InitShooter()"<< std::endl;
}

void InitShooter::Initialize() {
	if(!Robot::shooter->IsInitialized()){
		Robot::shooter->Initialize();
	}
}

bool InitShooter::IsFinished() {
	return Robot::shooter->TestIsInitialized();
}

void InitShooter::Execute() {
	Robot::shooter->Log();
}

void InitShooter::End() {
	if(!Robot::shooter->IsInitialized()){
		std::cout << "InitShooter:: Shooter Initialized"<< std::endl;
		Robot::shooter->SetInitialized();
	}
}
