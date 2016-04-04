/*
 * LoadBall.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: alpiner
 */

#include <Commands/LoadBall.h>
#include "Robot.h"

#define SET_LOW_DELAY 1

LoadBall::LoadBall() : Command("LoadBall") {
	Requires(Robot::loader.get());
	Requires(Robot::holder.get());
	state=LOW;
	elapsed_time=0;
	std::cout << "new LoadBall()"<< std::endl;
}

void LoadBall::SetLow() {
	std::cout << "LoadBall::SetLow()"<< std::endl;
	Robot::loader->StopRollers();
	Robot::loader->SetLow(); // goto lower limit switch
	state=LOW;
	elapsed_time=TimeSinceInitialized();
	SetTimeout(elapsed_time+SET_LOW_DELAY);
}

// first button push
void LoadBall::SetLoad() {
	if(Robot::holder->IsBallPresent()){
		std::cout << "LoadBall: Error - Ball is already loaded"<< std::endl;
		return;
	}
	std::cout << "LoadBall::SetLoad()"<< std::endl;
	Robot::loader->SpinRollers(true);
	Robot::loader->LoadBall();
	state=LOAD;
}

// ===========================================================================================================
// LoadBall::Initialize()
// - Called once for each trigger button push (Button 1) when in loading state
// ===========================================================================================================
void LoadBall::Initialize() {
	switch(state){
	default:
	case LOW:
		SetLoad();
		break;
	case LOAD:
		SetLow();
		break;
	}
}

// ===========================================================================================================
// LoadBall::IsFinished()
// -  Loader state machine
// ===========================================================================================================
bool LoadBall::IsFinished() {
	if(!Robot::loader->Loading()){
		std::cout << "LoadBall: aborted - shooter mode set"<< std::endl;
		return true;
	}
	switch(state){
	case LOW:
		if(IsTimedOut()){
			std::cout << "LoadBall: Timeout expired when setting low angle"<< std::endl;
			return true;
		}
		if(Robot::loader->LifterAtLowerLimit()){
			std::cout << "LoadBall: Lifter is at low angle"<< std::endl;
			return true;
		}
		break;
	case LOAD:
		if(Robot::holder->IsBallPresent()){
			std::cout << "LoadBall: Ball Was Successfully Loaded"<< std::endl;
			SetLow();
			return true;
		}
		break;
	}
	return false;
}

void LoadBall::End() {
}

void LoadBall::Interrupted() {
	std::cout << "LoadBall::Interrupted()"<< std::endl;
}
