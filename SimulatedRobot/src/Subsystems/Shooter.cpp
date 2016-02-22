/*
 * Shooter.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Subsystems/Shooter.h>

Shooter::Shooter(int m1, int m2, int m3) : Subsystem("Shooter"),
	angleMotor(m1), leftMotor(m2,false),rightMotor(m3,false)
{
	std::cout<<"New Shooter("<<m1<<","<<m2<<","<<m3<<")"<<std::endl;
	angle=0;
	SetLimits(0,50);
	angleMotor.SetPID(GPMotor::POSITION, 0.1, 0, 0);
	angleMotor.SetDistancePerPulse(RPD(1)); // 1 degree = 0.01745 radians
	angleMotor.SetPercentTolerance(5.0);
	angleMotor.SetSetpoint(0);
	angleMotor.Set(0);
	//angleMotor.Enable();
}
// Initialize
void Shooter::Init(){

}

// Shoot the ball
void Shooter::Shoot(){

}
// Set the shooter angle
void Shooter::SetTargetAngle(double a){
	a=a>max?max:a;
	a=a<min?min:a;
	angle=a;
	angleMotor.Set(angle);
}

double Shooter::GetTargetAngle(){
	return angle;
}

double Shooter::GetAngle(){
	return angleMotor.Get();
}

void Shooter::SetLimits(double a1, double a2){
	min=a1;
	max=a2;
}

