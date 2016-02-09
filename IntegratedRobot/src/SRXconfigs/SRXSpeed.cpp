/*
 * SRXSpeed.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#include <SRXconfigs/SRXSpeed.h>
#include <WPILib.h>
SRXSpeed::SRXSpeed(int id, double Pvalue, double Ivalue, double Dvalue, int a):CANTalon(id)
{
	SetControlMode(CANTalon::kSpeed);
	SetFeedbackDevice(CANTalon::QuadEncoder);
	SetP(Pvalue);
	SetI(Ivalue);
	SetD(Dvalue);
	EnableControl();
	maxTicks=a;
}

SRXSpeed::~SRXSpeed() {
	// TODO Auto-generated destructor stub
}

void SRXSpeed::ChangePID(float P, float I, float D)
{
	SetP(P);
	SetI(I);
	SetD(D);
}

void SRXSpeed::SetTargetSpeed(float speed)
{
	target=speed;
}
void SRXSpeed::Obey(){
	Set(target);
}

bool SRXSpeed::CloseEnough(float tolerance)
{
	bool current=GetEncVel();
	if(current>target-tolerance&&current<target+tolerance)
	{
		return true;
	}
	else{return false;}
}
