/*
 * ExpelBall.cpp
 *
 *  Created on: Mar 31, 2016
 *      Author: alpiner
 */

#include <Commands/CancelLoad.h>
#include "Robot.h"

#define CANCEL_TIMEOUT 1.0

CancelLoad::CancelLoad() : Command("CancelLoad") {
	Requires(Robot::loader.get());
	std::cout << "new CancelLoad()"<< std::endl;
}

void CancelLoad::Initialize() {
	std::cout << "CancelLoad Started ..."<< std::endl;
	Robot::loader->CancelLoad();
	SetTimeout(CANCEL_TIMEOUT);
}

bool CancelLoad::IsFinished() {
	if(IsTimedOut()){
		std::cout << "CancelLoad::Timeout reached"<< std::endl;
		return true;
	}
	if(Robot::loader->LifterAtLowerLimit()){
		std::cout << "CancelLoad::Cancel succeeded"<< std::endl;
		return true;
	}
	return false;
}

void CancelLoad::End() {
	std::cout << "CancelLoad::End()"<< std::endl;
}

void CancelLoad::Interrupted() {
	End();
}

void CancelLoad::Execute() {
	Robot::loader->GoToZeroLimitSwitch();
}
