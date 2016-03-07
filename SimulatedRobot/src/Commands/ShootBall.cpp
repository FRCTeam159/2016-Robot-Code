/*
 * ShootBall.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Commands/ShootBall.h>
#include "Robot.h"
#define GATE_DELAY 0.5
#define FLYWHEEL_DELAY 0.5
#define PUSH_DELAY 0.5
#define RESET_DELAY 0.5

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
	timeout_time=0;
	std::cout << "new ShootBall"<< std::endl;
}
void ShootBall::OpenGate(){
	std::cout << "Opening gate .."<<std::endl;
	state=OPENGATE;
	Robot::holder->OpenGate();
	timeout_time=GATE_DELAY;
	SetTimeout(timeout_time);
}
void ShootBall::TurnFlywheelsOn(){
	timeout_time+=FLYWHEEL_DELAY;
	Robot::shooter->Shoot(true);
	state=FLYWHEELS_ON;
	std::cout << "Turning flywheels on .."<<std::endl;
	SetTimeout(timeout_time);
}
void ShootBall::PushBall(){
	state=PUSHER_ON;
	std::cout << "PushBall started .."<<std::endl;
	Robot::holder->PushBall(true);
	timeout_time+=PUSH_DELAY;
	SetTimeout(timeout_time);
}
void ShootBall::ResetShooter(){
	state=RESET_ANGLE;
	std::cout << "Resetting shooter .."<<std::endl;
	double min=Robot::shooter->GetMinAngle();
	Robot::shooter->SetTargetAngle(min);
	Robot::holder->CloseGate();

	timeout_time+=RESET_DELAY;
	SetTimeout(timeout_time);
}

// Called just before this Command runs the first time
void ShootBall::Initialize() {
	timeout_time=0;
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
		if(timed_out)
			PushBall();
		break;
	case PUSHER_ON:
		if(timed_out){
			if(ball_present){
				std::cout << "Shoot Error (ball did not leave before timeout)"<<std::endl;
				return true;
			}
			else{
				std::cout << "Shot complete (ball ejected)"<<std::endl;
				ResetShooter();
			}
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
	std::cout << "ShootBall End"<<std::endl;
	Robot::shooter->Shoot(false);
	Robot::holder->PushBall(false);
}
// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ShootBall::Interrupted() {
	End();
}
