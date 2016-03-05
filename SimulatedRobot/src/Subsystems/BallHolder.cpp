/*
 * Holder.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Subsystems/BallHolder.h>

#define GATEMOTORSPEED 0.4
#define PUSHMOTORSPEED 1.0
#define PUSHHOLDSPEED 0.10


BallHolder::BallHolder(int m1, int m2) : Subsystem("Holder"),
	gateMotor(m1,true),pushMotor(m2,false),lowerLimit(0),upperLimit(1),ballSensor(2)
{
	gateopen=false;
	std::cout<<"New BallHolder("<<m1<<","<<m2<<")"<<std::endl;
}

bool BallHolder::IsGateOpen(){
	return upperLimit.Get();
}

bool BallHolder::IsGateClosed(){
	return lowerLimit.Get();
}

bool BallHolder::IsBallPresent(){
	double distance=ballSensor.GetAverageVoltage();
	std::cout<<"ballSensor:"<<distance<<std::endl;

	return distance<0.1?true:false;
}

void BallHolder::Init(){
	//pushMotor.SetPID(GPMotor::SPEED, 1.5, 0.001, 4);
	pushMotor.Set(PUSHHOLDSPEED);

}
void BallHolder::OpenGate(){
	gateopen=true;
	pushMotor.Set(PUSHHOLDSPEED);
	gateMotor.Set(GATEMOTORSPEED);
}
void BallHolder::CloseGate(){
	gateopen=false;
	pushMotor.Set(PUSHHOLDSPEED);
	gateMotor.Set(-GATEMOTORSPEED);
}
void BallHolder::PushBall(bool t){
	if(t)
		pushMotor.Set(PUSHMOTORSPEED);
	else
		pushMotor.Set(PUSHHOLDSPEED);
}
