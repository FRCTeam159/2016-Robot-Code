/*
 * Holder.cpp
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#include <Holder/Holder.h>
#define GATEMOTORSPEED 0.1
#define PUSHERMOTORSPEED 0.1


Holder::Holder(int mtr1,int mtr2,int ls1, int ls2, int IR)
: gateMotor(mtr1), pushMotor(mtr2),readyToFire(ls1),readyToLoad(ls2),IRsensor(IR){
	gateTicksPerRevolution = GATEMOTOR_ET*GATEMOTOR_GR;
	isAtForwardLimit=false;
	isAtReverseLimit=false;
#ifdef CANTALON_GATE
	gateMotor.ConfigEncoderCodesPerRev(gateTicksPerRevolution);
	gateMotor.SetControlMode(CANSpeedController::kPosition);
	gateMotor.ConfigLimitMode(CANSpeedController::kLimitMode_SoftPositionLimits);
	gateMotor.ConfigRevLimitSwitchNormallyOpen(true);
	gateMotor.ConfigFwdLimitSwitchNormallyOpen(true);
	gateMotor.Enable();
	gateMotor.SetPosition(0);
	gateMotor.ConfigSoftPositionLimits(gateTicksPerRevolution/4,0);
	gateMotor.SetPID(0.1, 0.001, 3);
	gateMotor.SetInverted(true);
	gateMotor.SetFeedbackDevice(CANTalon::QuadEncoder);
#endif

#ifdef CANTALON_PUSHER
#endif
	// TODO Auto-generated constructor stub
}
//===========================================
//void Holder::Init
//- call in teleop init
//===========================================
void Holder::Init(){
	isAtForwardLimit=false;
	isAtReverseLimit=false;
	gateMotor.SetControlMode(CANSpeedController::kCurrent);
	gateMotor.Set(-GATEMOTORSPEED);
}
//===========================================
//void Holder::FindZero
//- find reverse hard limit switch and set zero position
//- call from teleop periodic
//===========================================
void Holder::FindZero(){
	bool atTarget=gateMotor.IsRevLimitSwitchClosed();
	if(atTarget){
		gateMotor.SetPosition(0);
		isAtReverseLimit=true;
	}
}
void Holder::GrabBall(){

}
void Holder::AutoHold(){

}
//ready to load
void Holder::PushBall(){

}
bool Holder::IsLoaded(){
	return IRsensor.Get();
}
