/*
 * Holder.cpp
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#include <Holder/Holder.h>
#define GATEMOTORSPEED 0.1
#define PUSHERMOTORSPEED 0.1
#define BALLDETECTIONDELAY 1
#define SENSORTRIPPED 0
#define FORWARDLIMITPOSITION (gateTicksPerRevolution/4)
#define MAXERRORTICKS 100
#define ENCMULT 1988
#define P 0.001
#define I 0
#define D 0

Holder::Holder(int mtr1,int mtr2,int ls1, int ls2, int IR)
: gateMotor(mtr1), pushMotor(mtr2),revGateLimit(ls1),fwdGateLimit(ls2),IRsensor(IR){
	gateTicksPerRevolution = (double)GATEMOTOR_ET*GATEMOTOR_GR/ENCMULT;
	atForwardLimit=false;
	atReverseLimit=false;
	state=FIND_ZERO;
	ballDetectionDelay = BALLDETECTIONDELAY;
#ifdef CANTALON_GATE
	gateMotor.ConfigEncoderCodesPerRev(gateTicksPerRevolution);
	gateMotor.SetControlMode(CANSpeedController::kPosition);
	//gateMotor.ConfigLimitMode(CANSpeedController::kLimitMode_SoftPositionLimits);
	//gateMotor.Enable();
	//gateMotor.SetPosition(0);
	gateMotor.ConfigSoftPositionLimits(gateTicksPerRevolution/4,0);
	//gateMotor.SetInverted(true);
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
	//gateMotor.Disable();
	gateMotor.Enable();
	//gateMotor.SetExpiration(1000);
	gateMotor.SetInverted(true);
	gateMotor.ConfigRevLimitSwitchNormallyOpen(true);
	gateMotor.ConfigFwdLimitSwitchNormallyOpen(true);
	atForwardLimit=false;
	atReverseLimit=false;
	gateMotor.SetControlMode(CANSpeedController::kPercentVbus);
	gateMotor.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	gateMotor.Enable();
	gateMotor.Set(-GATEMOTORSPEED);
	printf("Holder Init, Motor:%d, Motor Inverted:%d\n",
			gateMotor.IsEnabled(),gateMotor.GetInverted());
}
//===========================================
//void Holder::FindZero
//- find reverse hard limit switch and set zero position
//- call from teleop periodic
//===========================================
void Holder::FindZero(){
	bool atTarget=gateMotor.IsRevLimitSwitchClosed();
	printf("looking for zero\n");
	if(atTarget && !atReverseLimit){
		printf("atTarget:%d\n",atTarget);
		gateMotor.Set(0);
		gateMotor.SetControlMode(CANSpeedController::kPosition);
		gateMotor.ConfigSoftPositionLimits(gateTicksPerRevolution/4,1);
		gateMotor.SetPosition(0);
		gateMotor.Set(0);
		gateMotor.SetPID(P,I,D);
		atReverseLimit=true;
		state=WAIT_FOR_BALL_TO_ENTER;
	}
	else if(!atReverseLimit)
	{
		//printf("looking for zero\n");
	}
}
void Holder::GrabBall(){

}
void Holder::AutoHold(){
	switch(state){
	case FIND_ZERO:
		FindZero();
		break;
	case WAIT_FOR_BALL_TO_ENTER:
		WaitForBallToEnter();
		break;
	case GO_TO_FORWARD_LIMIT:
		SetGateToForwardLimit();
		break;
	case WAIT_FOR_BALL_TO_LEAVE:
		WaitForBallToLeave();
		break;
	case GO_TO_REVERSE_LIMIT:
		SetGateToReverseLimit();
		break;
	}
}
//ready to load
void Holder::PushBall(){

}
bool Holder::IsLoaded(){
	return IRsensor.Get();
}

void Holder::Test(){
	FindZero();
}

void Holder::TestInit(){
	Init();
	//gateMotor.SetInverted(false);
}

void Holder::TeleopInit(){
	Init();
	state=FIND_ZERO;
}

void Holder::Disable()
{
//	gateMotor.Disable();
}

void Holder::TestPeriodic(){
	AutoHold();
//	printf("Motor Is Inverted:%d\n",gateMotor.GetInverted());
//	if(gateMotor.IsEnabled() == false){
//		printf("Motor Disabled, %d\n",gateMotor.GetInverted());
//	}
}

void Holder::TeleopPeriodic(){
	AutoHold();
}

void Holder::WaitForBallToEnter(){
	int ballDetected = IRsensor.Get();
	//printf("waiting for ball detection\n");
	if(ballDetected == SENSORTRIPPED){
		printf("WAIT_FOR_BALL_TO_ENTER ball is detected, going to forward limit\n");
		gateMotor.Set(0);
		//Wait(ballDetectionDelay);
		//gateMotor.Set(GATEMOTORSPEED);
		gateMotor.Set(FORWARDLIMITPOSITION);
		state=GO_TO_FORWARD_LIMIT;
	}
}

void Holder::WaitForBallToLeave(){
	int ballDetected = IRsensor.Get();
	if(ballDetected != SENSORTRIPPED){
		printf("WAIT_FOR_BALL_TO_LEAVE ball is not detected, going to reverse limit\n");
		//gateMotor.Set(0);
		//Wait(ballDetectionDelay);
		gateMotor.SetPID(P,I,D);
		state=GO_TO_REVERSE_LIMIT;
		//gateMotor.SetPosition(FORWARDLIMITPOSITION);
		//gateMotor.Set(-GATEMOTORSPEED);
		gateMotor.Set(0);
	}
}

void Holder::SetGateToForwardLimit(){
	bool atTarget = isAtForwardLimit();
	if(atTarget){
		printf("GO_TO_FORWARD_LIMIT at forward limit, waiting for ball to leave\n");
		state=WAIT_FOR_BALL_TO_LEAVE;
		//gateMotor.Set(0);
		//gateMotor.Disable();
		gateMotor.SetPID(0,0,0);
		//gateMotor.ClearIaccum();
	}
}

void Holder::SetGateToReverseLimit(){
	bool atTarget = isAtReverseLimit();
	if(atTarget){
		printf("GO_TO_REVERSE_LIMIT at reverse limit\n");
		state=WAIT_FOR_BALL_TO_ENTER;
		gateMotor.Set(0);
	}
}

bool Holder::isAtReverseLimit(){
	bool motionEnabled = gateMotor.GetReverseLimitOK();
	int position = gateMotor.GetEncPosition();
	//printf("position:%d motionEnabled:%d\n",position,motionEnabled);
	double error = position-0;
//	printf("rev position:%d rev error:%g real position:%g\n",
	//		position,error,gateMotor.Get());
	if(motionEnabled)
		return false;
	else
		return true;
}

bool Holder::isAtForwardLimit(){
	bool motionEnabled = gateMotor.GetForwardLimitOK();
	int position = gateMotor.GetEncPosition();
	//printf("position:%d motionEnabled:%d\n",position,motionEnabled);
	double error = position-FORWARDLIMITPOSITION;
	//printf("fwd position:%d fwd error:%g\n",position,error);
	if(motionEnabled == true)
		return false;
	else
		return true;
}













