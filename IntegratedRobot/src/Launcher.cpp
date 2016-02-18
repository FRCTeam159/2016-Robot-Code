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
	return(fabs(pid->GetAvgError())<tolerance);
}
void Launcher::Aim(float range)//takes horizontal range, in meters
{
	float term = sqrt(2*.295*9.81/(.5*1.001*.0507));//2* boulder weight, g, drag, air density, cross-sectional area repectively
	float dH= 2.77-.254;//delta height
	float Vv0=sqrt(pow(term, 2)*(pow(2.71828, dH*2*9.81/pow(term,2))-1));//2.7... is euler's constant
	float Tvmax= (term/9.81)*atan(Vv0/term);//time to vmax
	float Vh0=pow(term,2)*(pow(2.71828, 9.81*range/pow(term,2))-1)/(9.81*Tvmax);//initial Horiz. Vel.
	SetAngle(atan(Vv0/Vh0)*(180/3.1415));//calculate and set target angle
	float V0= sqrt(pow(Vh0,2)+pow(Vv0, 2));
	float r=.058;
	float rball = .127;
	float Iwheel=.004891;//the .168 below is ~ distance from flywheel center to center of shooter
	float targetSpeed= V0*((((.168)/Iwheel)*(.295/2))+(1/r));//target speed in rad/s
	targetSpeed*=900/(2*3.1415);//convert from radians to ticks
	SetTargetSpeed(targetSpeed);

}
