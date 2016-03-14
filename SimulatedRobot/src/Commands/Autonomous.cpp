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

Autonomous::Autonomous() : CommandGroup("Autonomous") {
	AddSequential(new CloseGate()); // pinch the ball
	AddSequential(new DriveStraight(6,0)); // go forward
	AddSequential(new Turn(-43)); // turn
	AddSequential(new StepShooterAngle(30)); // set angle
	AddSequential(new ShootBall()); // shoot
	AddSequential(new FullStop()); // end autonomous
}

