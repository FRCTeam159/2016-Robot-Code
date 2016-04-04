/*
 * InitHolder.cpp
 *
 *  Created on: Mar 29, 2016
 *      Author: alpiner
 */

#include <Commands/ExecHolder.h>
#include "Robot.h"

ExecHolder::ExecHolder() : Command("InitHolder")  {
	Requires(Robot::holder.get());
	std::cout << "new InitHolder()"<< std::endl;
}

void ExecHolder::Initialize() {
	std::cout << "ExecHolder:Initialize()"<< std::endl;

	if(!Robot::holder->IsInitialized())
		Robot::holder->Initialize();
}

bool ExecHolder::IsFinished() {
	return false;
}

void ExecHolder::End() {
	std::cout << "ExecHolder:End()"<< std::endl;
}

void ExecHolder::Execute() {
	Robot::holder->Execute();
}

void ExecHolder::Interrupted() {
	std::cout << "ExecHolder:Interrupted()"<< std::endl;
}
