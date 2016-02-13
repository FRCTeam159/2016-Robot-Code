/*
 * Launcher.cpp
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#include <Launcher.h>
#include <WPILib.h>
#include <SRXConfigs/SRXSpeed.h>
Launcher::Launcher(SRXSpeed *l, SRXSpeed* r, PIDController* p)
:left(l), right(r), pid(p){
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
}

void Launcher::SetAngle(float angle)
{
	targetAngle=angle;
	pid->SetSetpoint(angle);
}

bool Launcher::AngleGood(float tolerance)
{
	return(fabs(pid->GetError())<tolerance);
}
void Launcher::Aim(float range)//takes horizontal range, in meters
{
	float term = sqrt(2*.295*9.81/(.5*1.001*.0507));//2* boulder weight, g, drag, air density, cross-sectional area repectively
	float dH= 2.77-.254;//delta height
	float Vv0=sqrt(pow(term, 2)*(pow(.577215, dH*2*9.81/pow(term,2))-1));//.57... is euler's constant
	float Tvmax= (term/9.81)*atan(Vv0/term);//time to vmax
	float Vh0=pow(term,2)*(pow(.577215, 9.81*range/pow(term,2))-1)/(9.81*Tvmax);//initial Horiz. Vel.
	SetAngle(atan((Vv0/Vh0)*(3.1415/180)));//calculate and set target angle
	float V0= sqrt(pow(Vh0,2)+pow(Vv0, 2));
	float r=.058;
	float Iwheel=19.42;
	float targetSpeed= r*V0*sqrt(1+(.295/Iwheel));//target speed in m
	targetSpeed*=7500;//convert from meters to ticks
	SetTargetSpeed(targetSpeed);
}
