/*
 * Autonomous.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: alpiner
 */

#include <Commands/Autonomous.h>
#include <Commands/DriveStraight.h>
#include <Commands/CloseGate.h>
#include <Commands/StepShooterAngle.h>
#include <Commands/ShootBall.h>
#include <Commands/Turn.h>
#include <Commands/FullStop.h>
#include <Commands/InitLoader.h>
#include "Robot.h"

Autonomous::Autonomous() : CommandGroup("Autonomous") {
	Requires(Robot::drivetrain.get());
	//AddSequential(new LoaderGotoZero()); // lower lifter
	AddSequential(new CloseGate()); // pinch the ball
	AddSequential(new DriveStraight(6,0)); // go forward
	AddSequential(new StepShooterAngle(30)); // set angle
	AddSequential(new Turn(-43)); // turn
	AddSequential(new ShootBall()); // shoot
	AddSequential(new FullStop()); // end autonomous
}

void Autonomous::Interrupted() {
	std::cout << "Autonomous::Interruped"<<std::endl;
	_End();
}

void  Autonomous::Cancel(){
	std::cout << "Autonomous::Cancel"<<std::endl;
	_End();
	CommandGroup::Cancel();
	Robot::drivetrain->EndTravel();
}
