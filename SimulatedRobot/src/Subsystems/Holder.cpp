/*
 * Holder.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Subsystems/Holder.h>

BallHolder::BallHolder(int m1, int m2) : Subsystem("Holder"),
	gateMotor(m1),pushMotor(m2)
{
	gateopen=false;

}

bool BallHolder::GateIsOpen(){
	return gateopen;
}

void BallHolder::OpenGate(){

}
void BallHolder::CloseGate(){

}
void BallHolder::PushBall(bool t){

}
