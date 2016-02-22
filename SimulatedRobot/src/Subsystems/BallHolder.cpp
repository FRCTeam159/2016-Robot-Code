/*
 * Holder.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Subsystems/BallHolder.h>

#define GATEMOTORSPEED 0.1
BallHolder::BallHolder(int m1, int m2) : Subsystem("Holder"),
	gateMotor(m1),pushMotor(m2)
{
	gateopen=false;
	std::cout<<"New BallHolder("<<m1<<","<<m2<<")"<<std::endl;
}

bool BallHolder::GateIsOpen(){
	return gateopen;
}

void BallHolder::OpenGate(){
	gateMotor.Set(GATEMOTORSPEED);
	gateopen=true;
}
void BallHolder::CloseGate(){
	gateMotor.Set(-GATEMOTORSPEED);
	gateopen=false;
}
void BallHolder::PushBall(bool t){

}
