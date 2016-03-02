/*
 * Holder.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Subsystems/BallHolder.h>

#define GATEMOTORSPEED 0.3
#define PUSHMOTORSPEED 1.0
#define PUSHHOLDSPEED 0.10


BallHolder::BallHolder(int m1, int m2) : Subsystem("Holder"),
	gateMotor(m1,true),pushMotor(m2,false)
{
	gateopen=false;
	std::cout<<"New BallHolder("<<m1<<","<<m2<<")"<<std::endl;
}

bool BallHolder::GateIsOpen(){
	return gateopen;
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
