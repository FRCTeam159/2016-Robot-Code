/*
 * VerticalTargeter.cpp
 *
 *  Created on: Feb 5, 2016
 *      Author: alpin
 */

#include <VerticalTargeter.h>

VerticalTargeter::VerticalTargeter(Launcher* s, Lidar* l):siege(s), laser(l) {
	// TODO Auto-generated constructor stub

}

VerticalTargeter::~VerticalTargeter() {
	// TODO Auto-generated destructor stub
}

void VerticalTargeter::Obey()
{
	siege->Obey();
}

void VerticalTargeter::SetAngle(float angle)
{
	siege->SetAngle(angle);
}

void VerticalTargeter::SetSpeed(float speed)
{
	siege->SetTargetSpeed(speed);
}

bool VerticalTargeter::AngleGood(float tolerance)
{
	return(siege->AngleGood(tolerance));
}

bool VerticalTargeter::SpeedGood(float tolerance)
{
	return(siege->SpeedGood(tolerance));
}

int VerticalTargeter::GetRange()
{
	return(laser->GetDistance());
}
