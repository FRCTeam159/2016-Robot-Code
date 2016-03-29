/*
 * LoaderGotoZero.cpp
 *
 *  Created on: Mar 23, 2016
 *      Author: alpiner
 */

#include <Commands/InitLoader.h>
#include "Robot.h"

InitLoader::InitLoader() : Command("InitLoader") {
	Requires(Robot::loader.get());
	std::cout << "new InitLoader()"<< std::endl;
}

void InitLoader::Initialize() {
	if(!Robot::loader->IsInitialized()){
		Robot::loader->Initialize();
	}
}

bool InitLoader::IsFinished() {
	return Robot::loader->TestIsInitialized();
}

void InitLoader::End() {
	if(!Robot::loader->IsInitialized()){
		std::cout << "InitLoader: Loader Initialized"<< std::endl;
		Robot::loader->SetInitialized();
	}
}

void InitLoader::Execute() {
	Robot::loader->Log();
}
