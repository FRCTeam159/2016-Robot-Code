/*
 * ShootBall.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Commands/ShootBall.h>
#include "Robot.h"

ShootBall::ShootBall() : Command("ShootBall") {
	Requires(Robot::shooter.get());
	Requires(Robot::holder.get());
	state=0;
	std::cout << "new ShootBall"<< std::endl;
}
// Called just before this Command runs the first time
void ShootBall::Initialize() {
	Robot::shooter->Shoot(true);
	state=FLYWHEELS_ON;
	SetTimeout(1);
	std::cout << "Shoot start"<<std::endl;
}

// Called repeatedly when this Command is scheduled to run
void ShootBall::Execute() {
}
// Make this return true when this Command no longer needs to run execute()
bool ShootBall::IsFinished() {
	bool timed_out= IsTimedOut();
	if(!timed_out)
		return false;
	switch(state){
	case FLYWHEELS_ON:
		SetTimeout(2);
		state=PUSHER_ON;
		std::cout << "PushBall start"<<std::endl;
		Robot::holder->PushBall(true);
		return false;
	default:
	case PUSHER_ON:
		state=0;
		return true;
	}
}
// Called once after isFinished returns true
void ShootBall::End() {
	std::cout << "PushBall end"<<std::endl;
	Robot::shooter->Shoot(false);
	Robot::holder->PushBall(false);
	std::cout << "Shoot end"<<std::endl;
}
// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ShootBall::Interrupted() {
	End();
}
