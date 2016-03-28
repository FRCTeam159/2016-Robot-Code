/*
 * Holder.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */
#include "Assignments.h"
#include <Subsystems/BallHolder.h>

#define GATEMOTORSPEED 0.5
#define PUSHMOTORSPEED 1.0
#define PUSHHOLDSPEED 0.05
#define BALLDETECTVALUE 0.5

BallHolder::BallHolder() : Subsystem("Holder"),
	gateMotor(HOLDER_GATE,false),pushMotor(HOLDER_PUSH,false),
	lowerLimit(GATE_MIN),upperLimit(GATE_MAX),ballSensor(BALL_SENSOR)
{
	gateopen=false;
	std::cout<<"New BallHolder("<<HOLDER_GATE<<","<<HOLDER_PUSH<<")"<<std::endl;
}

void BallHolder::AutonomousInit(){
	Init();
}
void BallHolder::TeleopInit(){
	Init();
}
void BallHolder::DisabledInit(){
}

bool BallHolder::IsGateOpen(){
	return upperLimit.Get();
}

bool BallHolder::IsGateClosed(){
	return lowerLimit.Get();
}
// ===========================================================================================================
// BallHolder::IsBallPresent
// ===========================================================================================================
// - return true if the ball is in the holder
// - Problems in simulation mode
//   o for Gazebo simulation need to use an ultrasonic sensor (because IRSensor isn't supported yet)
//   o but sensor doesn't detect other surfaces of the robot (i.e. need some external object)
//   o also, if no object is detected the value returned is not changed from what was last detected (bug?)
//   o A problem is that if the ball is initially in the holder the sensor returns some value (e.g. 2.9)
//     but if the sensor is at a low angle (e.g. 0) the value is unchanged when the ball is ejected
//     (because no other object is detected when the ball isn't present)
//   o The workaround was to point the sensor downward and increase the radius so that it detects
//     the ground plane when the ball isn't present
// ===========================================================================================================
bool BallHolder::IsBallPresent(){
	double distance=ballSensor.GetAverageVoltage();
	//std::cout<<"ballSensor:"<<distance<<std::endl;
	return distance<BALLDETECTVALUE?true:false;
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
