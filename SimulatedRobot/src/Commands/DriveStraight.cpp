/*
 * DriveStraight.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: alpiner
 */

#include <Commands/DriveStraight.h>
#include "Robot.h"

static int count=1;
DriveStraight::DriveStraight(double d) : Command("DriveStraight")  {
	Requires(Robot::drivetrain.get());
    //pid = new PIDController(4, 0, 0, new DriveStraightPIDSource(),
    //		                         new DriveStraightPIDOutput());
	//pid=0;
    distance=d;
	std::cout << "new DriveStraight("<<distance<<")"<< std::endl;
}

// Called just before this Command runs the first time
// note: if pid is created in constructor it doesn't work (at least in simulation)
void DriveStraight::Initialize() {
//    Robot::drivetrain->Init();
//    if(pid)
//    	delete pid;
//    pid = new PIDController(0.3, 0.001, 2, this,this);
//    pid->SetAbsoluteTolerance(0.1);
//    pid->SetSetpoint(distance);
//    pid->Enable();
	Robot::drivetrain->SetDistance(distance);
	//std::cout << "DriveStraight Started : "<<pid->GetSetpoint()<<std::endl;
	std::cout << "DriveStraight Started"<<std::endl;

}
void DriveStraight::Execute() {
//	double l=Robot::drivetrain->GetLeftDistance();
//	double r=Robot::drivetrain->GetRightDistance();
//	double d=Robot::drivetrain->GetDistance();
//	std::cout << "DriveStraight "<<l<<","<<r<<","<<d<<")"<<std::endl;
	//std::cout<<"Target:"<<pid->GetSetpoint()<<" err:"<<pid->GetError()<<" cor:"<<pid->Get()<<std::endl;
}

bool DriveStraight::IsFinished() {
	//return pid->OnTarget();
	return Robot::drivetrain->OnTarget();

}

void DriveStraight::End() {
	//std::cout << "DriveStraight End"<<std::endl;
	// Stop PID and the wheels
	double l=Robot::drivetrain->GetLeftDistance();
	double r=Robot::drivetrain->GetRightDistance();
	double d=Robot::drivetrain->GetDistance();
	std::cout << "DriveStraight End("<<l<<","<<r<<","<<d<<")"<<std::endl;


	//pid->Disable();
	//Robot::drivetrain->Drive(0, 0);
}

//double DriveStraight::PIDGet(){
//	double d=Robot::drivetrain->GetDistance();
//	return d;
//}
//void DriveStraight::PIDWrite(float c) {
//	//std::cout << "DriveStraight PIDWrite:"<<d<<std::endl;
//	double d=Robot::drivetrain->GetDistance();
//	double l=Robot::drivetrain->GetLeftDistance();
//	double r=Robot::drivetrain->GetRightDistance();
//	double diff=(l-r);
//	double f=1;
//	double c1=c;
//	double c2=c;
//	if(count>2){
//	c1-=f*diff;
//	c2+=f*diff;
//	}
//
//	if((count%10)==0){
//	   std::cout << "DriveStraight PIDWrite:"<<c<<","<<l<<","<<r<<","<<d<<","<<diff<<")"<<std::endl;
//	}
//
//    Robot::drivetrain->Drive(c1, c2);
//    count++;
//}

//DriveStraightPIDSource::~DriveStraightPIDSource() {}
//double DriveStraightPIDSource::PIDGet() {
//	double d=Robot::drivetrain->GetDistance();
//	//std::cout << "DriveStraight PIDGet:"<<d<<std::endl;
//    return d;
//}
//
//DriveStraightPIDOutput::~DriveStraightPIDOutput() {}
//void DriveStraightPIDOutput::PIDWrite(float d) {
//	//std::cout << "DriveStraight PIDWrite:"<<d<<std::endl;
//
//    Robot::drivetrain->Drive(d, d);
//}
