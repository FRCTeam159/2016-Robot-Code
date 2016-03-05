/*
 * DriveStraight.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: alpiner
 */

#include <Commands/DriveStraight.h>
#include "Robot.h"

DriveStraight::DriveStraight(double d) : Command("DriveStraight")  {
	Requires(Robot::drivetrain.get());
    //pid = new PIDController(4, 0, 0, new DriveStraightPIDSource(),
    //		                         new DriveStraightPIDOutput());
    distance=d;
	std::cout << "new DriveStraight("<<distance<<")"<< std::endl;
}

// Called just before this Command runs the first time
// note: if pid is created in constructor it doesn't work (at least in simulation)
void DriveStraight::Initialize() {
    Robot::drivetrain->Reset();
    if(pid)
    	delete pid;
    pid = new PIDController(0.2, 0.001, 0.5, new DriveStraightPIDSource(),
    		                         new DriveStraightPIDOutput());
    pid->SetAbsoluteTolerance(0.1);
    pid->SetSetpoint(distance);
    pid->Enable();
	std::cout << "DriveStraight Started : "<<pid->GetSetpoint()<<std::endl;
}
void DriveStraight::Execute() {
	//std::cout<<"Target:"<<pid->GetSetpoint()<<" err:"<<pid->GetError()<<" cor:"<<pid->Get()<<std::endl;
}

bool DriveStraight::IsFinished() {
	return pid->OnTarget();
}

void DriveStraight::End() {
	std::cout << "DriveStraight End"<<std::endl;
	// Stop PID and the wheels
	pid->Disable();
	Robot::drivetrain->Drive(0, 0);
}

DriveStraightPIDSource::~DriveStraightPIDSource() {}
double DriveStraightPIDSource::PIDGet() {
	double d=Robot::drivetrain->GetDistance();
	//std::cout << "DriveStraight PIDGet:"<<d<<std::endl;
    return d;
}

DriveStraightPIDOutput::~DriveStraightPIDOutput() {}
void DriveStraightPIDOutput::PIDWrite(float d) {
	//std::cout << "DriveStraight PIDWrite:"<<d<<std::endl;

    Robot::drivetrain->Drive(d, d);
}
