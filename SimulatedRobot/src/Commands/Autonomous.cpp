/*
 * Autonomous.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: alpiner
 */

#include <Commands/Autonomous.h>
#include <Commands/DriveStraight.h>
#include <Commands/OpenGate.h>

#define METERS_PER_FOOT 3.28084
Autonomous::Autonomous() : CommandGroup("Autonomous") {
	AddSequential(new OpenGate()); // pinch the ball
	AddSequential(new DriveStraight(2*METERS_PER_FOOT)); // go forward 2 meters
}

