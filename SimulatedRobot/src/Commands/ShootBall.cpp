/*
 * ShootBall.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Commands/ShootBall.h>

ShootBall::ShootBall() : Command("ShootBall") {
	// TODO Auto-generated constructor stub

}
// Called just before this Command runs the first time
void ShootBall::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShootBall::Execute() {
	//Robot::drivetrain->Drive(Robot::oi->GetJoystick());
}
// Make this return true when this Command no longer needs to run execute()
bool ShootBall::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ShootBall::End() {
	//Robot::drivetrain->Drive(0, 0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ShootBall::Interrupted() {
	End();
}
