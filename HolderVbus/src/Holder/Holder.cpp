/*
 * Holder.cpp
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#include <Holder/Holder.h>
#include <time.h>
#include <sys/timeb.h>
#define GATEMOTORSPEED 0.2
#define BALLDETECTIONDELAY 2000
#define SENSORTRIPPED 0
#define FORWARDLIMITPOSITION (gateTicksPerRevolution/4)
#define MAXERRORTICKS 100
#define ENCMULT 1988
#define PUSHMOTORSPEED -1
#define P 0.1
#define I 0
#define D 0

//#define PUSH_EMULATION

Holder::Holder(int mtr1,int mtr2, int IR)
: gateMotor(mtr1), pushMotor(mtr2), IRsensor(IR){
	gateTicksPerRevolution = (double)GATEMOTOR_ET*GATEMOTOR_GR*4;
	atForwardLimit=false;
	atReverseLimit=false;
	state=FIND_ZERO;
	ballDetectionDelay = BALLDETECTIONDELAY;
	foundZero=false;
#ifdef CANTALON_GATE
//	gateMotor.ConfigEncoderCodesPerRev(gateTicksPerRevolution);
	gateMotor.SetControlMode(CANSpeedController::kPosition);
	gateMotor.SetFeedbackDevice(CANTalon::QuadEncoder);
	gateMotor.SetSensorDirection(true);
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
	if(foundZero == false){
		gateMotor.ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
	}
	else{
		gateMotor.SetPosition(0);
	}
	gateMotor.Set(-GATEMOTORSPEED);
	gateMotor.SetControlMode(CANSpeedController::kPercentVbus);
	gateMotor.ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	gateMotor.Enable();
	printf("Holder Init, Motor:%d, Motor Inverted:%d\n",
			gateMotor.IsEnabled(),gateMotor.GetInverted());
#endif

#ifdef CANTALON_PUSHER
	pushMotor.SetControlMode(CANSpeedController::kPercentVbus);
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
		gateMotor.ConfigSoftPositionLimits(FORWARDLIMITPOSITION,0);
		gateMotor.SetP(P);
		gateMotor.SetI(I);
		gateMotor.SetD(D);
		atReverseLimit=true;
		foundZero=true;
		state=WAIT_FOR_BALL_TO_ENTER;
	}
	else
	{
		gateMotor.Set(-GATEMOTORSPEED);
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
	case BALL_PUSH_ERR:
		RemoveBall();
		break;
	}
}
//ready to load
void Holder::PushBall(){
	if(state==WAIT_FOR_PUSH_REQUEST)
	{
		pushRequested = true;
		pushComplete = false;
	}
}
int Holder::CheckPushed(){
	return pushComplete;
}

bool Holder::IsLoaded(){
	return !IRsensor.Get();
}

void Holder::Test(){
	FindZero();
}

void Holder::TestInit(){
	Init();
}

//===========================================
//void Holder::TeleopInit
//===========================================
//- Starts the state machine
//===========================================
void Holder::TeleopInit(){
	Init();
	if(foundZero == false){
		state=FIND_ZERO;
	}
	else
	{
		if(IsAtReverseLimit())
			state = WAIT_FOR_BALL_TO_ENTER;
		else if(!IRsensor.Get())
		{
			state = WAIT_FOR_PUSH_REQUEST;
		}
		else
			state = GO_TO_REVERSE_LIMIT;
	}
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

//===========================================
//void Holder::AutonomousInit
//===========================================
//- First time check to see if FindZero has been called
//  then sets the limit mode to switches only and changes state
//===========================================
void Holder::AutonomousInit(){
	Init();
	if(foundZero==false){
		gateMotor.ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
	}
	state=WAIT_FOR_BALL_TO_ENTER;
}

void Holder::AutonomousPeriodic(){
	AutoHold();
}
//===========================================
//void Holder::WaitForBallToEnter
//===========================================
//- This State machine state : WAIT_FOR_BALL_TO_ENTER
//- Caller state machine state : GO_TO_REVERSE_LIMIT
//  or FIND_ZERO
//- Assumes gate is at reverse limit
//- Waits for IR sensor to detect ball
//- when ball is detected move gate to forward limit
//  and push ball into pusher wheel (push motor is off)
//===========================================
void Holder::WaitForBallToEnter(){
	int ballDetected = !IRsensor.Get();
	if(ballDetected){
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
//===========================================
//void Holder::SetGateToForwardLimit
//===========================================
//- This State machine state : GO_TO_FORWARD_LIMIT
//- Caller state machine state : WAIT_FOR_BALL_TO_ENTER
//- Waits for soft or hard limit
//- then go to next state and fake pushRequested
//===========================================
void Holder::SetGateToForwardLimit(){
	bool atTarget = IsAtForwardLimit();
	if(atTarget){
		int pos = gateMotor.GetEncPosition();
		printf("at forward limit(%d), waiting for ball to leave\n",pos);
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

//===========================================
//void Holder::WaitForPushRequest
//===========================================
//- This State machine state : WAIT_FOR_PUSH_REQUEST
//- Caller state machine state : GO_TO_FORWARD_LIMIT
//- Waits for a push request (pushRequested is faked)
//  then push ball with pushMotor and change state
//- start ftime function
//===========================================
void Holder::WaitForPushRequest(){
	if(pushRequested){
		printf("pushing ball to fly wheels\n");
		SetPushMotorSpeed(PUSHMOTORSPEED);
		pushRequested = false;
		state=WAIT_FOR_BALL_TO_LEAVE;
		ftime(&start_time);
	}
}

//===========================================
//void Holder::WaitForBallToLeave
//===========================================
//- This State machine state : WAIT_FOR_BALL_TO_LEAVE
//- Caller state machine state : WAIT_FOR_PUSH_REQUEST
//- Wait for the IR sensor to stop detecting the ball
//  then end ftime function, and use output as a delay
//- Set gateMotor to reverse speed
//===========================================
void Holder::WaitForBallToLeave(){
	ftime(&end_time);
	double delt=deltaTime(&start_time, &end_time);
	//printf("WAIT_FOR_BALL_TO_LEAVE ball is not detected delt=%g\n",delt);
	if(delt >= ballDetectionDelay){
		int ballDetected = !IRsensor.Get();
		if(ballDetected)
		{
			pushComplete = PUSH_ERROR;
			std::cout<<"Holder: push failed!"<<std::endl;
			SetPushMotorSpeed(.3);
			gateMotor.Set(-.35);
			state= BALL_PUSH_ERR;
		}
		else
		{
			pushComplete = PUSH_COMPLETE;
			printf("going to reverse limit\n");
			SetPushMotorSpeed(0);
			gateMotor.Set(-GATEMOTORSPEED);
			state=GO_TO_REVERSE_LIMIT;
			//gateMotor.Set(0);
			//Wait(ballDetectionDelay);
			//gateMotor.SetPID(P,I,D);
			//gateMotor.SetPosition(FORWARDLIMITPOSITION);
			//gateMotor.Set(0);
		}


	}
}

void Holder::RemoveBall()
{
	bool atTarget= IsAtReverseLimit();
	if(atTarget){
		gateMotor.Set(0);
		SetPushMotorSpeed(0);
		state=WAIT_FOR_BALL_TO_ENTER;
	}
	else
	{
		gateMotor.Set(-.35);
	}

}
//===========================================
//void Holder::SetGateToReverseLimit
//===========================================
//- This State machine state : GO_TO_REVERSE_LIMIT
//- Caller state machine state : WAIT_FOR_BALL_TO_LEAVE
//- If FindZero has not been called, call FindZero
//- Otherwise find reverse limit,
//  then stop motor and go to next state in the state machine
//===========================================
void Holder::SetGateToReverseLimit(){
	bool atTarget = IsAtReverseLimit();
	if(foundZero==false){
		state=FIND_ZERO;
		return;
	}
	if(atTarget){
		printf("GO_TO_REVERSE_LIMIT at reverse limit\n");
		state=WAIT_FOR_BALL_TO_ENTER;
		SetPushMotorSpeed(0);
		gateMotor.Set(0);
	}
	else
	{
		gateMotor.Set(-GATEMOTORSPEED);
	}
}

//===========================================
//void Holder::IsAtReverseLimit
//===========================================
//- Checks if the soft limit has been reached
//  or if the reverse limit switch is closed
//===========================================
bool Holder::IsAtReverseLimit(){
	bool atTarget=gateMotor.IsRevLimitSwitchClosed();
	if(atTarget == true)
		return true;
	else
		return false;
}

//===========================================
//void Holder::IsAtForwardLimit
//===========================================
//- Checks if the soft limit has been reached
//  or if the forward limit switch is closed
//===========================================
bool Holder::IsAtForwardLimit(){
	bool motionEnabled = gateMotor.GetForwardLimitOK();
	bool atTarget=gateMotor.IsFwdLimitSwitchClosed();
	if((motionEnabled == false) || atTarget == true)
		return true;
	else
		return false;
}

//===========================================
//void Holder::SetPushMotorSpeed
//===========================================
//- Sets pushMotor speed with a double.
//===========================================
void Holder::SetPushMotorSpeed(double speed){
#ifdef CANTALON_PUSHER
	pushMotor.Set(speed);
#endif
}

//===========================================
//void Holder::deltaTime
//===========================================
//- Used to measure time elapsed through code.
//===========================================
int Holder::deltaTime(struct timeb* first, struct timeb* after){
	int diff =after->time-first->time;
	int mdiff= after->millitm-first->millitm;
	return((1000*diff)+mdiff);
}




