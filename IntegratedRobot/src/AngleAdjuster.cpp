/*
 * AngleAdjuster.cpp
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#include <AngleAdjuster.h>
#include <WPILib.h>
#include <SRXConfigs/SRXPosition.h>
AngleAdjuster::AngleAdjuster(SRXPosition* a, float b):motor(a), ticksPerDegree(b) {
	// TODO Auto-generated constructor stub
	targetAngle=0;
}

AngleAdjuster::~AngleAdjuster() {
	// TODO Auto-generated destructor stub
}


void AngleAdjuster::SetAngle(float newTarget)//give this angle in degrees
{
	targetAngle=newTarget;
}

void AngleAdjuster::Obey()
{
	motor->Set(targetAngle*ticksPerDegree);

}

bool AngleAdjuster::CloseEnough(int acceptableError)
{
	int currentPos=motor->GetPosition();
	if(currentPos<(targetAngle*ticksPerDegree)+acceptableError&&currentPos>(targetAngle*ticksPerDegree)-acceptableError)
	{
		return(true);
	}
	else
	{
		return(false);
	}
}

float AngleAdjuster::GetCurrentAngle()
{
	return(motor->GetEncPosition()/ticksPerDegree)*(6.282/360);
}
