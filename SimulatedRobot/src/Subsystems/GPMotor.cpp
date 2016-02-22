/*
 * SmartMotor.cpp
 *
 *  Created on: Jul 6, 2015
 *      Author: alpiner
 */

#include <Subsystems/GPMotor.h>

GPMotor::GPMotor(int id) : GPMotor(id,true){
}

#if MOTORTYPE == CANTALON
GPMotor::GPMotor(int id, int enc) : CANTalon(id){
	control_mode=SPEED;
	inverted=false;
	syncGroup=0x08;
}
#else
#if MOTORTYPE == VICTOR
GPMotor::GPMotor(int id,bool enc) : Victor(id){
#else
GPMotor::GPMotor(int id,bool enc) : Talon(id){
#endif
	pid=0;
	encoder=0;
	if(enc){
		int ival=(id-1)*2+1; // 1,3,5,..
		encoder=new Encoder(ival,ival+1); // {1,2} {3,4} {5,6} ..
	}
	control_mode=SPEED;
	inverted=false;
	syncGroup=0x08;
}
#endif


GPMotor::~GPMotor(){
#if MOTORTYPE != CANTALON
#ifdef ENCODER
	if(encoder)
		delete encoder;
#endif
	if(pid)
		delete pid;
#endif
}

void GPMotor::UsePIDOutput(double value){
#if MOTORTYPE == CANTALON
	CANTalon::Set(value,syncGroup);
#elif MOTORTYPE == VICTOR
	Victor::PIDWrite(value);
#else
	Talon::PIDWrite(value);
#endif
}
double GPMotor::ReturnPIDInput(){
	if(control_mode==SPEED)
		return GetVelocity();
	else
		return GetDistance();
}
double GPMotor::PIDGet(){
	return ReturnPIDInput();
}

void GPMotor::SetVelocity(double value){
#if MOTORTYPE == CANTALON
	CANTalon::Set(value);
#else
	Talon::Set(value);
#endif
}

double GPMotor::GetVelocity(){
#if MOTORTYPE == CANTALON
	return CANTalon::GetSpeed();
#else

	return encoder->GetRate();
#endif

}

void GPMotor::SetDistance(double value){
#if MOTORTYPE == CANTALON
	CANTalon::SetPosition(value);
#else
	if(pid)
		pid->SetSetpoint(value);
	else
		std::cout<<"ERROR SetDistance:PID=NULL"<<std::endl;
#endif
}

double GPMotor::GetDistance(){
#if MOTORTYPE == CANTALON
	return CANTalon::GetPosition();
#else
	return encoder->GetDistance();
#endif
}

void GPMotor::ClearIaccum(){
#ifdef REAL
	CANTalon::ClearIaccum();
#else
	if(pid){
		pid->Reset(); // clears accumulator but also disables
		pid->Enable();
	}
#endif

}

void GPMotor::Enable(){
#if MOTORTYPE == CANTALON
	CANTalon::EnableControl();
#else
	if(pid)
		pid->Enable();
	else
		std::cout<<"ERROR Enable:PID=NULL"<<std::endl;
#endif

}
void GPMotor::Disable(){
#if MOTORTYPE == CANTALON
	CANTalon::Disable();
#else
	Talon::Disable();
	if(pid)
		pid->Disable();
#endif
}

void GPMotor::SetPID(int mode, double P, double I, double D){
	SetPID(P,I,D);
	SetMode(mode);
}
void GPMotor::SetInverted(bool t) {
	inverted=t;
}

void GPMotor::SetPID(double P, double I, double D){
#if MOTORTYPE != CANTALON
	if(pid)
		delete pid;
	pid=new PIDController(P, I, D,this,this);
#else
	CANTalon::SetPID(P,I,D);
#endif
}
void GPMotor::SetMode(int m){
	if((m & POSITION) && (m != SPEED) )
		m=SPEED;
#if MOTORTYPE == CANTALON
	CANTalon::ControlMode mode = (m==POSITION)? CANTalon::ControlMode::kPosition: CANTalon::ControlMode::kSpeed;
	CANTalon::SetControlMode(mode);
#else
	if(pid)
		pid->SetPIDSourceType((m==POSITION)? PIDSourceType::kDisplacement:PIDSourceType::kRate);
//	encoder->SetPIDSourceParameter((m==POSITION)? PIDSource::kDistance:PIDSource::kRate);
#endif
	control_mode=m;
}

void GPMotor::SetSetpoint(double value){
#if MOTORTYPE == CANTALON
	if(control_mode==SPEED)
		CANTalon::Set(value);
	else
		CANTalon::SetPosition(value);
#else
	if(pid)
		pid->SetSetpoint(value);
	else
		std::cout<<"ERROR SetSetpoint:PID=NULL"<<std::endl;
#endif
}

void GPMotor::SetInputRange(double min, double max){
#if MOTORTYPE == CANTALON
	// TODO: what is the equivalent function for a CANTalon ?
#else
	if(pid)
		pid->SetInputRange(min,max);
#endif

}
void GPMotor::SetOutputRange(double min, double max){
#if MOTORTYPE == CANTALON
	// TODO: what is the equivalent function for a CANTalon ?
#else
	if(pid)
		pid->SetOutputRange(min,max);
#endif
}

void GPMotor::SetPercentTolerance(double tol){
#if MOTORTYPE == CANTALON
	// TODO: implement equivalent function for a CANTalon
#else
	if(pid)
		pid->SetPercentTolerance(tol);
#endif
}

bool GPMotor::OnTarget(){
#if MOTORTYPE == CANTALON
	// TODO: implement equivalent function for a CANTalon
#else
	if(pid)
		return pid->OnTarget();
	else
		return false;
#endif
}

void GPMotor::Reset(){
#if MOTORTYPE == CANTALON
	// TODO: implement equivalent function for a CANTalon
#else
#ifdef ENCODER
	if(encoder)
		encoder->Reset();
#endif
#endif
}
void GPMotor::SetDistancePerPulse(double target){
#if MOTORTYPE == CANTALON
	// TODO: implement equivalent function for a CANTalon
#else
#ifdef ENCODER
	if(encoder)
		encoder->SetDistancePerPulse(target);
#endif
#endif
}
