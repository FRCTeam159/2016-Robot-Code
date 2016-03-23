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
	SetTimeout(INIT_TIMEOUT);
	if(!Robot::loader->IsInitialized()){
		std::cout << "InitLoader initializing Loader"<< std::endl;
		Robot::loader->Initialize();
	}
}

bool InitLoader::IsFinished() {
	if(IsTimedOut()){
		std::cout << "InitLoader Error: Timeout expired"<<std::endl;
		return true;
	}
	return Robot::loader->AtLowerLimit();
}

void InitLoader::End() {
	if(!Robot::loader->IsInitialized()){
		std::cout << "InitLoader::End()"<< std::endl;
		Robot::loader->SetInitialized();
	}
}
