/*
 * Loader.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#include <Loader.h>
#include <WPILib.h>
#include <AngleAdjuster.h>
Loader::Loader(AngleAdjuster* a, Victor* b):liftMotor(a), rollerMotor(b) {
	//TODO Auto-generated contructtor stub
}

Loader::~Loader() {
	// TODO Auto-generated destructor stub
}

bool Loader::SetAngle(float target)
{
	return(liftMotor->CloseEnough(200));
}

void Loader::SpinRoller()
{
	rollerMotor->Set(.8);
}
