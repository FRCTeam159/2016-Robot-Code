/*
 * Loader.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#include <Loader/Loader.h>
#include <WPILib.h>


Loader::Loader(int a, int b, I2C::Port p):liftMotor(a), rollerMotor(b), accel(p) {
	//TODO Auto-generated contructtor stub
	liftMotor.ConfigRevLimitSwitchNormallyOpen(true);
	liftMotor.SetControlMode(CANTalon::kPercentVbus);
	liftMotor.ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
	sAngCtrl= new PIDController(1,0,0, &accel, &liftMotor);
}

Loader::~Loader() {
	// TODO Auto-generated destructor stub
	delete sAngCtrl;
}

void Loader::SetAngle(float angle){

}
void Loader::SpinRoller(bool){

}
void Loader::TeleopInit(){
	liftMotor.Set(0.1);//The motor will be directly controlled by the PID anyway, no?
	sAngCtrl->Enable();//-Joseph
	sAngCtrl->SetPID(1,0,0);
	sAngCtrl->SetSetpoint(30);
}
void Loader::TeleopPeriodic(){
	float angle=accel.PIDGet();

	float error=sAngCtrl->GetError();
	printf("angle:%g error:%g\n",angle,error);
	if (angle > 45) {
		liftMotor.Set(0);//calling liftMotor->Set() directly doesn't do anything while
		//PID loop is running. consider calling PIDController::Disable() first
		sAngCtrl->SetPID(0,0,0);//I would just call Disable() here -Joseph
	}
}
void Loader::AutonomousInit(){

}
void Loader::AutonomousPeriodic(){

}
void Loader::Obey(){
	switch(state){
	case SETLOWPOSITON:
		break;
	case GRABBALL:
		break;
	}
}
void Loader::setLowPosition(){

}
void Loader::grabBall(){

}
