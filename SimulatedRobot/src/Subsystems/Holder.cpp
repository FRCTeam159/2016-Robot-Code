/*
 * Holder.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */
#include <Commands/ExecHolder.h>
#include <Subsystems/Holder.h>
#include "Assignments.h"

#define GATEMOTORSPEED 0.2
#define PUSHMOTORSPEED 1.0
#define PUSHHOLDSPEED 0.1

#define BALLDETECTVALUE 0.5

Holder::Holder() : Subsystem("Holder"),
	gateMotor(HOLDER_GATE,false),pushMotor(HOLDER_PUSH,false),
	lowerLimit(GATE_MIN),upperLimit(GATE_MAX),ballSensor(BALL_SENSOR)
{
	std::cout<<"New BallHolder("<<HOLDER_GATE<<","<<HOLDER_PUSH<<")"<<std::endl;
	Log();
}

void Holder::Log() {
	SmartDashboard::PutBoolean("Holder Initialized", IsInitialized());
	SmartDashboard::PutBoolean("Gate Open", IsGateOpen());
	SmartDashboard::PutBoolean("Gate Closed", IsGateClosed());
	SmartDashboard::PutBoolean("Ball present", IsBallPresent());
}

void Holder::InitDefaultCommand() {
	SetDefaultCommand(new ExecHolder());
}

void Holder::Init(){
	initialized=false;
	pushMotor.Set(0);
	gateMotor.Set(0);
}

void Holder::AutonomousInit(){
	Init();
}
void Holder::TeleopInit(){
	Init();
}
void Holder::DisabledInit(){
	Init();
}

bool Holder::IsGateOpen(){
	return upperLimit.Get();
}

bool Holder::IsGateClosed(){
	return lowerLimit.Get();
}
// ===========================================================================================================
// Holder::IsBallPresent
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
bool Holder::IsBallPresent(){
	double distance=ballSensor.GetAverageVoltage();
	//std::cout<<"ballSensor:"<<distance<<std::endl;
	return distance<BALLDETECTVALUE?true:false;
}

void Holder::OpenGate(){
	gateMotor.Set(GATEMOTORSPEED);
}
void Holder::CloseGate(){
	gateMotor.Set(-GATEMOTORSPEED);
}
void Holder::PushBall(bool t){
	if(t)
		pushMotor.Set(PUSHMOTORSPEED);
	else
		pushMotor.Set(PUSHHOLDSPEED);
}

bool Holder::IsInitialized() {
	return initialized;
}

void Holder::SetInitialized() {
	initialized=true;
	gateMotor.Set(0);
}

void Holder::Initialize() {
	if(!IsGateClosed())
		gateMotor.Set(-GATEMOTORSPEED);
}

// ===========================================================================================================
// Holder::Execute
// ===========================================================================================================
// Called repeatedly from Default Command (ExecHolder) in Teleop Mode
// - if !initialized, goto lower limit switch (close gate)
// - else if ball is present pinch ball (open gate)
// ===========================================================================================================
void Holder::Execute() {
	Log();
	if(!initialized){
		if(IsGateClosed())
			SetInitialized();
		else
			gateMotor.Set(-GATEMOTORSPEED);
	}
	else if(IsBallPresent()){
		gateMotor.Set(GATEMOTORSPEED);
		pushMotor.Set(PUSHHOLDSPEED);
	}
}
