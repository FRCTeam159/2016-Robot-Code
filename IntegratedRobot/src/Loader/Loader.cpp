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
	sAngCtrl->Enable();
}
void Loader::TeleopPeriodic(){
	sAngCtrl->SetSetpoint(30);
	if (sAngCtrl->Get() > 45) {
		sAngCtrl->Disable();
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
