/*
 * SmartMotor.cpp
 *
 *  Created on: Jul 6, 2015
 *      Author: alpiner
 */
#include "WPILib.h"

#include <Subsystems/GPMotor.h>

#ifdef SIMULATION

#define SIMRATE 0.01


MyPIDController::MyPIDController(float Kp, float Ki, float Kd,PIDSource *source, PIDOutput *output)
								: PIDController(Kp, Ki, Kd, 0.0f, source, output,SIMRATE)
{
	tolerance=0.05;
}
void MyPIDController::Calculate()
{
	PIDController::Calculate();
//#define DEBUG_PID
#ifdef DEBUG_PID
	if(IsEnabled()){
		double s=GetSetpoint();
		double e=GetError();
		double c=Get();
		bool b=IsEnabled();
		bool t=OnTarget();
		std::cout<<"PID enabled:"<<b<<" Target:"<<s<<" err:"<<e<<" cor:"<<c<<" OnTarget:"<<t<<std::endl;
	}
#endif
}
// BUG in PIDController :
// - Uses a buffer (m_buf) to average previous error values
// - but m_buf never gets written to so "OnTarget" always returns false
bool MyPIDController::OnTarget(){
	//return PIDController::OnTarget();
	if(!IsEnabled())
		return false;
	double e=GetError();
	double delta=fabs(e);
	return delta<=tolerance;
}
// need to set tolerance manually to allow custom OnTarget calculation
void MyPIDController::SetAbsoluteTolerance(double d){
	tolerance=d;
}
// BUG in PIDController :
// - using the native version of this function causes NaN errors
double MyPIDController::CalculateFeedForward(){
	return 0;
}
#endif

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
	if(encoder)
		delete encoder;
	if(pid)
		delete pid;
}
//#define DEBUG_OUTPUT
void GPMotor::PIDWrite(float output){
	if (output !=output) {
		std::cout<< GetChannel() << " (NaN)" << std::endl;
		return;
	}
#ifdef DEBUG_OUTPUT
	std::cout << GetChannel()<<" "<<output<<std::endl;
#endif
#if MOTORTYPE == CANTALON
	CANTalon::PIDWrite(output);
#elif MOTORTYPE == VICTOR
	Victor::PIDWrite(output);
#else
	Talon::PIDWrite(output);
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
	CANTalon::Enable();
#else
	if(pid)
		pid->Enable();
	else
		std::cout<<"ERROR Enable:PID=NULL"<<std::endl;
#endif
}

bool GPMotor::IsEnabled(){
#if MOTORTYPE == CANTALON
	return CANTalon::IsEnabled();
#else
	if(pid)
		return pid->IsEnabled();
	else
		return false;
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
	//if(encoder)
	//	encoder->SetReverseDirection(t);
}

void GPMotor::SetContinuous(bool b){
	if(pid)
		pid->SetContinuous(b);
}
void GPMotor::SetPID(double P, double I, double D){
#if MOTORTYPE != CANTALON
	if(pid)
		delete pid;
	pid=new MyPIDController(P, I, D,this,this);
#else
	CANTalon::SetPID(P,I,D);
#endif
}
void GPMotor::SetMode(int m){
	if((m != POSITION) && (m != SPEED) )
		m=SPEED;
#if MOTORTYPE == CANTALON
	CANTalon::ControlMode mode = (m==POSITION)? CANTalon::ControlMode::kPosition: CANTalon::ControlMode::kSpeed;
	CANTalon::SetControlMode(mode);
#else
	SetPIDSourceType((m==POSITION)? PIDSourceType::kDisplacement:PIDSourceType::kRate);
	if(encoder)
		encoder->SetPIDSourceType((m==POSITION)? PIDSourceType::kDisplacement:PIDSourceType::kRate);
#endif
	control_mode=m;
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

void GPMotor::SetTolerance(double tol){
#if MOTORTYPE == CANTALON
	// TODO: implement equivalent function for a CANTalon
#else
	if(pid)
		pid->SetAbsoluteTolerance(tol);
#endif
}

double GPMotor::GetTargetCorrection(){
#if MOTORTYPE == CANTALON
	// TODO: implement equivalent function for a CANTalon
#else
	if(pid)
		return pid->Get();
	else
		return 0;
#endif
}

double GPMotor::GetTargetError(){
#if MOTORTYPE == CANTALON
	// TODO: implement equivalent function for a CANTalon
#else
	if(pid)
		return pid->GetError();
	else
		return 0;
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
	if(encoder)
		encoder->Reset();
	if(pid)
		pid->Reset();

	//ClearIaccum();
#endif
}
void GPMotor::SetDistancePerPulse(double target){
#if MOTORTYPE == CANTALON
	// TODO: implement equivalent function for a CANTalon
#else
	if(encoder)
		encoder->SetDistancePerPulse(target);
#endif
}
