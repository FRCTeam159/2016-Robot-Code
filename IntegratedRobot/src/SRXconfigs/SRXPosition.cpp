/*
 * SRXPosition.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#include <SRXconfigs/SRXPosition.h>
#include <WPILib.h>
SRXPosition::SRXPosition(int id, double p, double i, double d, bool invert):CANTalon(id) {
	SetControlMode(CANTalon::kPosition);
	SetP(p);
	SetI(i);
	SetD(d);
	SetFeedbackDevice(CANTalon::QuadEncoder);
	invertMotor=invert;
	SetInverted(invert);
	if (invert)
	{
		ConfigFwdLimitSwitchNormallyOpen(false);
	}
	else
	{
		ConfigRevLimitSwitchNormallyOpen(false);
	}
	SetPosition(0);
	target=0;
	EnableControl();
}

SRXPosition::~SRXPosition() {
}

void SRXPosition::ZeroSelf()
{
	if(!IsFwdLimitSwitchClosed())
	{
		target+=50;
	}
	else
	{
		target=0;
		SetPosition(0);
	}
}
