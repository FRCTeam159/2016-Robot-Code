/*
 * Holder.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Subsystems/BallHolder.h>

#define GATEMOTORSPEED 1
#define PUSHMOTORSPEED 1

BallHolder::BallHolder(int m1, int m2) : Subsystem("Holder"),
	gateMotor(m1,false),pushMotor(m2,false)
{
	gateopen=true;
	std::cout<<"New BallHolder("<<m1<<","<<m2<<")"<<std::endl;
}

bool BallHolder::GateIsOpen(){
	return gateopen;
}

void BallHolder::OpenGate(){
	gateopen=true;
	gateMotor.Set(GATEMOTORSPEED);
}
void BallHolder::CloseGate(){
	gateopen=false;
	gateMotor.Set(-GATEMOTORSPEED);
}
void BallHolder::PushBall(bool t){
	if(t)
		pushMotor.Set(PUSHMOTORSPEED);
	else
		pushMotor.Set(0);
}