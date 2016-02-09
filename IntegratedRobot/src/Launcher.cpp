/*
 * Launcher.cpp
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#include <Launcher.h>
#include <WPILib.h>
#include <SRXConfigs/SRXSpeed.h>
Launcher::Launcher(SRXSpeed *l, SRXSpeed* r, AngleAdjuster* a)
:left(l), right(r), angler(a){
}

Launcher::~Launcher() {
	// TODO Auto-generated destructor stub
}

bool Launcher::SpeedGood(float tolerance)
{
	bool leftGood=left->CloseEnough(tolerance);
	bool rightGood=right->CloseEnough(tolerance);
	return(leftGood&&rightGood);
}

void Launcher::SetTargetSpeed(float speed)
{
	targetSpeed=speed;
}

void Launcher::Obey()
{
	left->SetTargetSpeed(targetSpeed);
	right->SetTargetSpeed(targetSpeed);
	angler->SetAngle(targetAngle);
}

void Launcher::SetAngle(float angle)
{
	targetAngle=angle;
}

bool Launcher::AngleGood(float tolerance)
{
	return(angler->CloseEnough(tolerance));
}
