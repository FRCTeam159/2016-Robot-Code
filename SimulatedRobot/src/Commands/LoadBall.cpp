/*
 * LoadBall.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: alpiner
 */

#include <Commands/LoadBall.h>
#include "Robot.h"

#define SET_MED_DELAY 1
#define SET_HIGH_DELAY 2
#define SET_LOW_DELAY 1
#define FIND_LOW_DELAY 2

enum {
	FIND_LOW=1,
	SET_MED=2,
	SET_HIGH=3,
	SET_LOW=4,
};

LoadBall::LoadBall() : Command("LoadBall") {
	Requires(Robot::loader.get());
	state=0;
	elapsed_time=0;
	std::cout << "new LoadBall()"<< std::endl;
}

void LoadBall::Initialize() {
	std::cout << "LoadBall::Initialize()"<< std::endl;
	elapsed_time=0;

}
bool LoadBall::IsFinished() {
	bool timed_out= IsTimedOut();
	switch(state){
	case FIND_LOW:
		break;
	case SET_MED:
		break;
	case SET_HIGH:
		break;
	case SET_LOW:
		break;
	}
	return true;
}

void LoadBall::FindLow() {
}

void LoadBall::SetLow() {
}

void LoadBall::SetMedium() {
}

void LoadBall::SetHigh() {
}

void LoadBall::End() {
	std::cout << "LoadBall::End()"<< std::endl;
}
