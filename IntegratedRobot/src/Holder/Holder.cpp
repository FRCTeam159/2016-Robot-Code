/*
 * Holder.cpp
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#include <Holder/Holder.h>
#include <time.h>
#include <sys/timeb.h>
#define GATEMOTORSPEED 0.1
#define PUSHERMOTORSPEED 0.1
#define BALLDETECTIONDELAY 2000
#define SENSORTRIPPED 0
#define FORWARDLIMITPOSITION (gateTicksPerRevolution/4)
#define MAXERRORTICKS 100
#define ENCMULT 1988
#define PUSHMOTORSPEED 20
#define P 0.1
#define I 0
#define D 0

#define PUSH_EMULATION

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
	pushRequested = false;
	pushComplete = false;
	atForwardLimit=false;
	atReverseLimit=false;
#ifdef CANTALON_GATE
	gateMotor.Enable();
	gateMotor.ConfigRevLimitSwitchNormallyOpen(true);
	gateMotor.ConfigFwdLimitSwitchNormallyOpen(true);
	gateMotor.ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
	gateMotor.SetControlMode(CANSpeedController::kPercentVbus);
	gateMotor.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	gateMotor.Enable();
	gateMotor.SetPosition(0);
	gateMotor.Set(-GATEMOTORSPEED);
	printf("Holder Init, Motor:%d, Motor Inverted:%d\n",
			gateMotor.IsEnabled(),gateMotor.GetInverted());
#endif

#ifdef CANTALON_PUSHER
	pushMotor.SetControlMode(CANSpeedController::kSpeed);
	pushMotor.ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);

#endif
}
//===========================================
//void Holder::FindZero
//- find reverse hard limit switch and set zero position
//- call from teleop periodic
//===========================================
void Holder::FindZero(){
	bool atTarget=gateMotor.IsRevLimitSwitchClosed();
	if(atTarget && !atReverseLimit){
		printf("found zero\n");
		gateMotor.Set(0);
		//gateMotor.SetControlMode(CANSpeedController::kPosition);
		gateMotor.Enable();
		gateMotor.SetPosition(0);
		gateMotor.Set(0);
		gateMotor.ConfigLimitMode(CANSpeedController::kLimitMode_SoftPositionLimits);
		gateMotor.ConfigSoftPositionLimits(496,0);
		gateMotor.SetPID(P,I,D);
		atReverseLimit=true;
		state=WAIT_FOR_BALL_TO_ENTER;
	}
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
	case WAIT_FOR_PUSH_REQUEST:
		WaitForPushRequest();
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
	pushRequested = true;
	pushComplete = false;
}
bool Holder::IsLoaded(){
	return IRsensor.Get();
}

void Holder::Test(){
	FindZero();
}

void Holder::TestInit(){
	Init();
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
}

void Holder::TeleopPeriodic(){
	AutoHold();
}

void Holder::WaitForBallToEnter(){
	int ballDetected = IRsensor.Get();
	if(ballDetected == SENSORTRIPPED){
		printf("WAIT_FOR_BALL_TO_ENTER ball is detected, going to forward limit\n");
		//gateMotor.Set(0);
		//Wait(ballDetectionDelay);
		gateMotor.SetControlMode(CANSpeedController::kPercentVbus);
		//gateMotor.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
		gateMotor.Enable();
		gateMotor.Set(GATEMOTORSPEED);
		//gateMotor.Set(FORWARDLIMITPOSITION);
		state=GO_TO_FORWARD_LIMIT;
	}
}

void Holder::SetGateToForwardLimit(){
	bool atTarget = isAtForwardLimit();
	if(atTarget){
		printf("GO_TO_FORWARD_LIMIT at forward limit, waiting for ball to leave\n");
		state=WAIT_FOR_PUSH_REQUEST;
#ifdef PUSH_EMULATION
		pushRequested=true;
#endif
		//gateMotor.Set(0);
		//gateMotor.Disable();
		//gateMotor.SetPID(0,0,0);
		//gateMotor.ClearIaccum();
	}
}

void Holder::WaitForPushRequest(){
	if(pushRequested){
		printf("pushing ball to fly wheels\n");
		SetPushMotorSpeed(PUSHMOTORSPEED);
		pushRequested = false;
		state=WAIT_FOR_BALL_TO_LEAVE;
		ftime(&start_time);
	}
}

void Holder::WaitForBallToLeave(){
	int ballDetected = IRsensor.Get();
	if(ballDetected != SENSORTRIPPED){
		ftime(&end_time);
		double delt=deltaTime(&start_time, &end_time);
		//printf("WAIT_FOR_BALL_TO_LEAVE ball is not detected delt=%g\n",delt);
		if(delt >= ballDetectionDelay){
			pushComplete = true;
			printf("going to reverse limit\n");
			//gateMotor.Set(0);
			//Wait(ballDetectionDelay);
			//gateMotor.SetPID(P,I,D);
			state=GO_TO_REVERSE_LIMIT;
			//gateMotor.SetPosition(FORWARDLIMITPOSITION);
			gateMotor.Set(-GATEMOTORSPEED);
			//gateMotor.Set(0);
		}
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
	bool atTarget=gateMotor.IsFwdLimitSwitchClosed();
	if((motionEnabled == false) || atTarget == true)
		return true;
	else
		return false;
}

bool Holder::isAtForwardLimit(){
	bool motionEnabled = gateMotor.GetForwardLimitOK();
	bool atTarget=gateMotor.IsFwdLimitSwitchClosed();
	if((motionEnabled == false) || atTarget == true)
		return true;
	else
		return false;
}

void Holder::SetPushMotorSpeed(double speed){
#ifdef CANTALON_PUSHER
	pushMotor.Set(speed);
#endif
}

bool Holder::CheckPushed(){
	return pushComplete;
}

int Holder::deltaTime(struct timeb* first, struct timeb* after){
	int diff =after->time-first->time;
	int mdiff= after->millitm-first->millitm;
	return((1000*diff)+mdiff);
}




