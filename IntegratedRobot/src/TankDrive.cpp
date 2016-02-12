/*
 * TankDrive.cpp
 *
 *  Created on: Jan 29, 2016
 *      Author: alpin
 */

#include <TankDrive.h>
#include <WPILib.h>
#include <SRXConfigs/SRXSlave.h>
TankDrive::TankDrive(CANTalon* a, CANTalon* b, SRXSlave* c, SRXSlave* d, int e):leftMotor(a),
rightMotor(b), leftSlave(c), rightSlave(d), maxTicks(e){
}

TankDrive::~TankDrive() {
	// TODO Auto-generated destructor stub
}

void TankDrive::ArcadeDrive(Joystick* stick)
{
	float xAxis = deadband(stick->GetX(), .2);
	float yAxis = deadband(-1*stick->GetY(), .2);
	float zAxis = deadband(stick->GetZ(), .3);

	float left=0;
	float right=0;

	if(!zAxis==0)
	{
		left=zAxis;
		right=zAxis;
	}

	else if(!xAxis==0)
	{
		if(xAxis<0)
		{
			left=(fabs(yAxis)-fabs(xAxis))*(yAxis/fabs(yAxis));
			right=yAxis;
		}
		if(xAxis>0)
		{
			right=(fabs(yAxis)-fabs(xAxis))*(yAxis/fabs(yAxis));
			left=yAxis;
		}
	}
	else if(!yAxis==0)
	{
		left=yAxis;
		right=yAxis;
	}
	leftTarget=left*maxTicks;
	rightTarget=right*maxTicks;
}

float TankDrive::deadband(float value, float deadzone)
{
	if(fabs(value)<deadzone)
	{
		return(0);
	}
	else
	{
		return(value);
	}
}

void TankDrive::SetPosTargets(float left, float right)
{
	leftTarget=left;
	rightTarget=right;
}

void TankDrive::ConfigAuto(float p, float i, float d)
{
	leftMotor->SetControlMode(CANTalon::kPosition);
	leftMotor->SetP(p);
	leftMotor->SetI(i);
	leftMotor->SetD(d);
	leftMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	leftMotor->EnableControl();
	rightMotor->SetControlMode(CANTalon::kPosition);
	rightMotor->SetP(p);
	rightMotor->SetI(i);
	rightMotor->SetD(d);
	rightMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	rightMotor->EnableControl();
	config=1;
}
void TankDrive::ConfigTeleop(float p, float i, float d)
{
	leftMotor->SetControlMode(CANTalon::kSpeed);
	leftMotor->SetP(p);
	leftMotor->SetI(i);
	leftMotor->SetD(d);
	leftMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	leftMotor->EnableControl();
	rightMotor->SetControlMode(CANTalon::kSpeed);
	rightMotor->SetP(p);
	rightMotor->SetI(i);
	rightMotor->SetD(d);
	rightMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	rightMotor->EnableControl();
	config=2;
}
void TankDrive::Obey()
{
	leftMotor->Set(leftTarget);
	rightMotor->Set(rightTarget);
	leftSlave->Obey();
	rightSlave->Obey();
}

bool TankDrive::CloseEnough(float tolerance)
{
	int currentLeft;
	int currentRight;
	if(config==0)
	{
		std::cout<<"not configured!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
	}
	if(config==1)
	{
		currentLeft=leftMotor->GetEncPosition();
		currentRight=rightMotor->GetEncPosition();
	}
	if(config==2)
	{
		currentLeft=leftMotor->GetEncVel();
		currentRight=rightMotor->GetEncVel();
	}
	bool leftGood=(currentLeft>leftTarget-tolerance&&currentLeft<leftTarget+tolerance);
	bool rightGood=(currentRight>rightTarget-tolerance&&currentRight<rightTarget+tolerance);
	return(leftGood&&rightGood);
}

void TankDrive::PIDWrite(float output)
{
	leftTarget=output*-1;
	rightTarget=output;
	previousPID=output;
}

void TankDrive::ConfigForPID()
{
	leftMotor->SetControlMode(CANTalon::kPercentVbus);
	rightMotor->SetControlMode(CANTalon::kPercentVbus);
}
