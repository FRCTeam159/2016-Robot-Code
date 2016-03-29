/*
 * ShootBall.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Commands/ShootBall.h>
#include "Robot.h"
#define GATE_DELAY 2
#define FLYWHEEL_DELAY 2
#define PUSH_DELAY 1
#define RESET_DELAY 0.1

enum {
	OPENGATE=1,
	FLYWHEELS_ON=2,
	PUSHER_ON=3,
	RESET_ANGLE=4,
};

ShootBall::ShootBall() : Command("ShootBall") {
	Requires(Robot::shooter.get());
	Requires(Robot::holder.get());
	state=0;
	elapsed_time=0;
	std::cout << "new ShootBall"<< std::endl;
}
void ShootBall::OpenGate(){
	state=OPENGATE;
	elapsed_time=TimeSinceInitialized();
	std::cout<< elapsed_time << " Opening gate .."<<std::endl;
	Robot::holder->OpenGate();
	SetTimeout(elapsed_time+GATE_DELAY);
}
void ShootBall::TurnFlywheelsOn(){
	Robot::shooter->EnableFlywheels();
	state=FLYWHEELS_ON;
	elapsed_time=TimeSinceInitialized();
	std::cout<<elapsed_time << " Gate Open, Turning flywheels on .."<<std::endl;
	SetTimeout(elapsed_time+FLYWHEEL_DELAY);
}
void ShootBall::PushBall(){
	state=PUSHER_ON;
	elapsed_time=TimeSinceInitialized();
	double speed=Robot::shooter->GetSpeed();
	std::cout<<elapsed_time << " Flywheel speed="<<speed<<" PushBall started .."<<std::endl;
	Robot::holder->PushBall(true);
	SetTimeout(elapsed_time+PUSH_DELAY);
}
void ShootBall::ResetShooter(){
	state=RESET_ANGLE;
	elapsed_time=TimeSinceInitialized();
	std::cout<<elapsed_time << " Shot Complete, Resetting shooter .."<<std::endl;
	SetTimeout(elapsed_time+RESET_DELAY);
	double min=Robot::shooter->GetMinAngle();
	Robot::shooter->SetTargetAngle(min);
	Robot::holder->CloseGate();
}

// Called just before this Command runs the first time
void ShootBall::Initialize() {
	elapsed_time=0;
	std::cout << "ShootBall Started"<<std::endl;

	if(Robot::holder->IsBallPresent()){
		if(Robot::holder->IsGateClosed())
			OpenGate();
		else
			TurnFlywheelsOn();
	}
	else
		std::cout << "Shoot Aborted (no ball detected)"<<std::endl;
}

// Called repeatedly when this Command is scheduled to run
void ShootBall::Execute() {
}
// Make this return true when this Command no longer needs to run execute()
bool ShootBall::IsFinished() {
	bool timed_out= IsTimedOut();
	bool ball_present=Robot::holder->IsBallPresent();
	bool gate_open=Robot::holder->IsGateOpen();

	switch(state){
	case OPENGATE:
		if(timed_out && !gate_open){
			std::cout << "Shoot Error (gate did not open before timeout)"<<std::endl;
			return true;
		}
		if(gate_open)
			TurnFlywheelsOn();
		break;
	case FLYWHEELS_ON:
		if(timed_out){
			std::cout << "Shoot Error (flywheels not at speed before timeout)"<<std::endl;
			return true;
		}
		if(Robot::shooter->IsAtSpeed()){
			PushBall();
		}
		break;
	case PUSHER_ON:
		if(timed_out){
			if(ball_present){
				std::cout << "Shoot Error (ball did not leave before timeout)"<<std::endl;
				return true;
			}
			else
				ResetShooter();
		}
		break;
	case RESET_ANGLE:
		if(timed_out){
			state=0;
			return true;
		}
	}
	return false;
}
// Called once after isFinished returns true
void ShootBall::End() {
	elapsed_time=TimeSinceInitialized();
	std::cout<< elapsed_time << " ShootBall End"<<std::endl;
	Robot::shooter->DisableFlywheels();
	Robot::holder->PushBall(false);
}
// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ShootBall::Interrupted() {
	End();
}
