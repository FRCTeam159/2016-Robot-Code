/*
 * LoaderGotoZero.cpp
 *
 *  Created on: Mar 23, 2016
 *      Author: alpiner
 */

#include <Commands/InitLoader.h>
#include "Robot.h"

#define INIT_TIMEOUT 2

InitLoader::InitLoader() : Command("InitLoader") {
	Requires(Robot::loader.get());
	std::cout << "new InitLoader()"<< std::endl;
}

void InitLoader::Initialize() {
	if(!Robot::loader->IsInitialized()){
//		SetTimeout(INIT_TIMEOUT);
		//std::cout << "InitLoader initializing Loader"<< std::endl;
		Robot::loader->Initialize();
	}
}

bool InitLoader::IsFinished() {
//	if(!Robot::loader->IsInitialized() &&  IsTimedOut()){
//		return true;
//	}
	if(!Robot::loader->IsInitialized())
		return Robot::loader->LifterTestLowerLimit();
	return true;
}

void InitLoader::End() {
	if(!Robot::loader->IsInitialized() && Robot::loader->LifterAtLowerLimit()){
		std::cout << "InitLoader Loader at lower limit"<< std::endl;
		Robot::loader->SetInitialized();
	}
}

void InitLoader::Execute() {
	Robot::loader->Log();
}
