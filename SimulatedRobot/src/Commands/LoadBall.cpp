/*
 * LoadBall.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: alpiner
 */

#include <Commands/LoadBall.h>
#include "Robot.h"

LoadBall::LoadBall() : Command("LoadBall") {
	Requires(Robot::loader.get());
	std::cout << "new LoadBall()"<< std::endl;
}

void LoadBall::Initialize() {
	std::cout << "LoadBall::Initialize()"<< std::endl;
}
bool LoadBall::IsFinished() {
	return true;
}
void LoadBall::End() {
	std::cout << "LoadBall::End()"<< std::endl;
}
