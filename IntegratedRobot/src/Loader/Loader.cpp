/*
 * Loader.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#include <Loader/Loader.h>
#include <WPILib.h>
#define SETZEROSPEED -0.4
#define ROLLERMOTORSPEED 1
#define ANGLE 30
#define MINIMUM_ANGLE_ERROR 1
#define MINIMUM_TIMEOUT 1000


//end goal is two state:
//state 1: roller goes to 30 and stops, rolls motors until state 2
//state 2: roller motors stop and roller goes to limit switch

Loader::Loader(int a, int b, I2C::Port p):liftMotor(a), rollerMotor(b), accel(p) {
	liftMotor.ConfigRevLimitSwitchNormallyOpen(true);
	liftMotor.SetControlMode(CANTalon::kPercentVbus);
	liftMotor.ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
	sAngCtrl= new PIDController(0.005,0,0, &accel, &liftMotor);
	targetAngle = ANGLE;
	state = WAITING;
	atLimit = false;
	oldState = state;
	timeoutTime = MINIMUM_TIMEOUT;
}

Loader::~Loader() {
	delete sAngCtrl;
}

void Loader::Obey(){
	switch(state){
	case SETLOW:
		//GoToZeroLimitSwitch();
		break;
	case WAITING:
		ftime(&end_time);
		if(deltaTime(&start_time, &end_time) < timeoutTime){
			state=SETLOW;
		}
		break;
	case SETHIGH:

		break;
	}
}

void Loader::SetAngle(float a){
	targetAngle=a;
}
void Loader::SpinRoller(bool){

}
void Loader::TeleopInit(){
	//liftMotor.Set(0.1);//The motor will be directly controlled by the PID anyway, no?
	//sAngCtrl->Enable();//-Joseph
	sAngCtrl->SetPID(0.005,0,0);
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
}
void Loader::AutonomousInit(){

}
void Loader::AutonomousPeriodic(){

}

void Loader::SetLow(){
	GoToZeroLimitSwitch();
	state=SETLOW;
	sAngCtrl->Disable();
	TurnRollersOn(false);
}
void Loader::SetHigh(){
	state=SETHIGH;
	sAngCtrl->Enable();
	sAngCtrl->SetSetpoint(targetAngle);
	TurnRollersOn(true);
}

//Stop state: Disables the PID Controller to stop liftMotor
//also stops the roller motor.
void Loader::Wait(){
	state=WAITING;
	ftime(&start_time);
}

void Loader::GoToZeroLimitSwitch(){
	liftMotor.Set(SETZEROSPEED);
}

void Loader::TurnRollersOn(bool on){
	if(on){
		rollerMotor.Set(ROLLERMOTORSPEED);
	}
	else{
		rollerMotor.Set(0);
	}
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
	atLimit = liftMotor.IsRevLimitSwitchClosed();
	return atLimit;
}

int Loader::deltaTime(struct timeb* first, struct timeb* after){
	int diff =after->time-first->time;
	int mdiff= after->millitm-first->millitm;
	return((1000*diff)+mdiff);
}





