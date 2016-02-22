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
}
// Called just before this Command runs the first time
void ShootBall::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShootBall::Execute() {
}
// Make this return true when this Command no longer needs to run execute()
bool ShootBall::IsFinished() {
	return false;
}
// Called once after isFinished returns true
void ShootBall::End() {
}
// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ShootBall::Interrupted() {
	End();
}
