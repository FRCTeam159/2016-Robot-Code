/*
 * Loader.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#include <Loader/Loader.h>
#include <WPILib.h>
#define SETZEROSPEED -0.6
#define ROLLERMOTORSPEED 1
#define MED_ANGLE 7
#define HIGH_ANGLE 17.5
#define MINIMUM_ANGLE_ERROR 1
#define MINIMUM_TIMEOUT 5000


//end goal is two state:
//state 1: roller goes to 30 and stops, rolls motors until state 2
//state 2: roller motors stop and roller goes to limit switch

Loader::Loader(int a, int b, I2C::Port p):liftMotor(a), rollerMotor(b), accel(p) {
	liftMotor.ConfigFwdLimitSwitchNormallyOpen(true);
	liftMotor.SetControlMode(CANTalon::kPercentVbus);
	liftMotor.ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
	liftMotor.SetInverted(true);
	sAngCtrl= new PIDController(0.1,0,0, &accel, &liftMotor);
	targetAngle = MED_ANGLE;
	state = HIGH;
	atLimit = false;
	oldState = state;
	timeoutTime = MINIMUM_TIMEOUT;
}

Loader::~Loader() {
	delete sAngCtrl;
}

void Loader::Obey(){
	switch(state){
	case LOW:
		//GoToZeroLimitSwitch();
		if(!atLimit){
			if(liftMotor.GetForwardLimitOK())
				liftMotor.Set(SETZEROSPEED);
			else
			{
				atLimit = true;
				liftMotor.Set(0);
				std::cout<<"loader::at bottom"<<std::endl;
			}
		}
		break;
	case HIGH:
		ftime(&end_time);
		if(deltaTime(&start_time, &end_time) > timeoutTime){
			SetLow();
			std::cout<<"time hit!";
		}
		break;
	case MED:

		break;
	case MANUAL:
		liftMotor.Set(ManualTarget);
		break;
	}
}

void Loader::SetAngle(float a){
	targetAngle=a;
}
void Loader::SpinRollers(bool forward){
	if(forward)
		rollerMotor.Set(ROLLERMOTORSPEED);
	else
		rollerMotor.Set(-ROLLERMOTORSPEED);
}
void Loader::TeleopInit(){
	//liftMotor.Set(0.1);//The motor will be directly controlled by the PID anyway, no?
	//sAngCtrl->Enable();//-Joseph
	//sAngCtrl->SetSetpoint(30);
}
void Loader::TeleopPeriodic(){

	/*float angle=accel.PIDGet();
	float error=sAngCtrl->GetError();
	printf("angle:%g error:%g\n",angle,error);
	if (angle > 45) {
		liftMotor.Set(0);//calling liftMotor->Set() directly doesn't do anything while
		//PID loop is running. consider calling PIDController::Disable() first
		sAngCtrl->SetPID(0,0,0);//I would just call Disable() here -Joseph
	}*/
	Obey();
}
void Loader::AutonomousInit(){

}
void Loader::AutonomousPeriodic(){

}
void Loader::DisabledInit()
{
	sAngCtrl->Disable();
}
void Loader::SetLow(){
	GoToZeroLimitSwitch();
	atLimit = false;
	state=LOW;
	sAngCtrl->Disable();
	StopRollers();
}
void Loader::SetMed(){
	state=MED;
	sAngCtrl->Enable();
	sAngCtrl->SetSetpoint(MED_ANGLE);
	SpinRollers(true);
}

//Stop state: Disables the PID Controller to stop liftMotor
//also stops the roller motor.
void Loader::SetHigh(){
	state=HIGH;
	sAngCtrl->SetSetpoint(HIGH_ANGLE);
	ftime(&start_time);
}

void Loader::SetManual()
{
	ManualTarget = 0;
	state = MANUAL;
	sAngCtrl->Disable();
}

void Loader::GoToZeroLimitSwitch(){
	liftMotor.Set(SETZEROSPEED);
}

void Loader::StopRollers()
{
	rollerMotor.Set(0);
}

bool Loader::AtGrabAngle(){
	float angle=accel.PIDGet();
	if(fabs(angle-targetAngle) < MINIMUM_ANGLE_ERROR){
		return true;
	}
	else
	{
		return false;
	}
}

bool Loader::AtZeroAngle(){
	atLimit = liftMotor.IsFwdLimitSwitchClosed();
	return atLimit;
}

int Loader::deltaTime(struct timeb* first, struct timeb* after){
	int diff =after->time-first->time;
	int mdiff= after->millitm-first->millitm;
	return((1000*diff)+mdiff);
}
void Loader::Continue()
{
	switch(state){
	case LOW:
		SetMed();
		break;
	case MED:
		SetHigh();
		break;
	}
}
void Loader::ExpelBall()
{
	state = CANCEL;
	sAngCtrl->SetSetpoint(HIGH_ANGLE);
	SpinRollers(false);
}
void Loader::Cancel(){
	switch(state){
	case MED:
		ExpelBall();
		break;
	case HIGH:
		ExpelBall();
		break;
	case CANCEL:
		SetLow();
		break;
	}
}

void Loader::SetManualPower(float power)
{
	ManualTarget = power;
}
