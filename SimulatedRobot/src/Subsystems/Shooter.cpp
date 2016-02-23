/*
 * Shooter.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Subsystems/Shooter.h>

#define FWSPEED 400

Shooter::Shooter(int m1, int m2, int m3) : Subsystem("Shooter"),
	angleMotor(m1,true), leftMotor(m2,true),rightMotor(m3,true)
{
	std::cout<<"New Shooter("<<m1<<","<<m2<<","<<m3<<")"<<std::endl;
	angle=0;
	angleMotor.SetPID(GPMotor::POSITION, 3, 0, 0);
	SetLimits(0,70);
	angleMotor.SetDistancePerPulse(1); // 1 degree = 0.01745 radians
	//angleMotor.SetPercentTolerance(1.0);
	angleMotor.SetSetpoint(0);
	//angleMotor.Set(1.0);
	angleMotor.Reset();
	angleMotor.Enable();


	leftMotor.SetPID(GPMotor::SPEED, 0.4, 0, 0);
	leftMotor.SetDistancePerPulse(RPD(1)); // 1 degree = 0.01745 radians
	rightMotor.SetPID(GPMotor::SPEED, 0.4, 0, 0);
	rightMotor.SetDistancePerPulse(RPD(1)); // 1 degree = 0.01745 radians
}

// Initialize
void Shooter::Init(){
	angleMotor.SetSetpoint(0);
	angleMotor.Enable();
	leftMotor.SetVelocity(0);
	rightMotor.SetVelocity(0);
}

// Shoot the ball
void Shooter::Shoot(bool t){
	double speed=t?FWSPEED:0;
	leftMotor.SetVelocity(speed);
	rightMotor.SetVelocity(speed);
}
// Set the shooter angle
void Shooter::SetTargetAngle(double a){
	//a=a>max?max:a;
	//a=a<min?min:a;
	angle=a;
	angleMotor.Enable();
	angleMotor.SetDistance(angle);
	//std::cout << "Shooter angle enabled:"<< angleMotor.IsEnabled() <<std::endl;
	//angleMotor.UsePIDOutput(angle);
	//angleMotor.Set(angle);
}

bool Shooter::OnTarget(){
	return angleMotor.OnTarget();
}
double Shooter::GetTargetAngle(){
	return angle;
}
double Shooter::GetTargetError(){
//	if(!angleMotor.IsEnabled())
//	std::cout << "Shooter angle not enabled"<<std::endl;

	return angleMotor.GetTargetError();
}
double Shooter::GetTargetCorrection(){
	return angleMotor.GetTargetCorrection();
}

double Shooter::GetAngle(){
	return angleMotor.GetDistance();
}

void Shooter::SetLimits(double a1, double a2){
	min=a1;
	max=a2;
	angleMotor.SetOutputRange(-100,100);
	angleMotor.SetInputRange(-100,100);


}

