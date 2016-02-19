/*
 * Loader.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#include <Loader/Loader.h>
#include <WPILib.h>
#define SETZEROSPEED -0.1
#define ROLLERMOTORSPEED 1
#define ANGLE 30
#define MINIMUM_ANGLE_ERROR 1


//end goal is two state:
//state 1: roller goes to 30 and stops, rolls motors until state 2
//state 2: roller motors stop and roller goes to limit switch

Loader::Loader(int a, int b, I2C::Port p):liftMotor(a), rollerMotor(b), accel(p) {
	liftMotor.ConfigRevLimitSwitchNormallyOpen(true);
	liftMotor.SetControlMode(CANTalon::kPercentVbus);
	liftMotor.ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
	sAngCtrl= new PIDController(0.5,0,0, &accel, &liftMotor);
	targetAngle = ANGLE;
	state = WAITING;
	atLimit = false;
}

Loader::~Loader() {
	delete sAngCtrl;
}

void Loader::Obey(){
	switch(state){
	case SETLOWPOSITION:
		GoToZeroLimitSwitch();
		break;
	case WAITING:
		Waiting();
		break;
	case GRABBALL:
		GrabbingBall();
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
	sAngCtrl->SetPID(1,0,0);
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

void Loader::SetLowPosition(){
	state=SETLOWPOSITION;
	sAngCtrl->Disable();
}
void Loader::GrabBall(){
	state=GRABBALL;
	sAngCtrl->Enable();
	sAngCtrl->SetSetpoint(targetAngle);
}

//Stop state: Disables the PID Controller to stop liftMotor
//also stops the roller motor.
void Loader::Waiting(){
	state=WAITING;
	sAngCtrl->Disable();
	rollerMotor.Set(0);
}

void Loader::GoToZeroLimitSwitch(){
	rollerMotor.Set(0);
	liftMotor.Set(SETZEROSPEED);
}

void Loader::GrabbingBall(){
	rollerMotor.Set(ROLLERMOTORSPEED);
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








