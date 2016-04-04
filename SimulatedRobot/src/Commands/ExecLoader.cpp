/*
 * LoaderGotoZero.cpp
 *
 *  Created on: Mar 23, 2016
 *      Author: alpiner
 */

#include <Commands/ExecLoader.h>
#include "Robot.h"

ExecLoader::ExecLoader() : Command("InitLoader") {
	Requires(Robot::loader.get());
	std::cout << "new InitLoader()"<< std::endl;
}

void ExecLoader::Initialize() {
	if(!Robot::loader->IsInitialized())
		Robot::loader->Initialize();
}

bool ExecLoader::IsFinished() {
	return false;
}

void ExecLoader::End() {
	std::cout << "ExecLoader:End()"<< std::endl;
}

void ExecLoader::Execute() {
	Robot::loader->Execute();
}

void ExecLoader::Interrupted() {
	End();
}
