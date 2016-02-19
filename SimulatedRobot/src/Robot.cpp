
#include "Robot.h"

DriveTrain* Robot::drivetrain = NULL;

OI* Robot::oi = NULL;


enum motorIDs {
	DRIVE_LEFT = 1,
	DRIVE_RIGHT =2,
	HOLDER_GATE =3,
	HOLDER_PUSH=4,
	SHOOTER_ANGLE=5,
	SHOOTER_LEFT=6,
	SHOOTER_RIGHT=7,
};

void Robot::RobotInit() {
	oi = new OI();
	drivetrain = new DriveTrain();
	holder=new BallHolder(HOLDER_GATE,HOLDER_PUSH);
	shooter=new Shooter(SHOOTER_ANGLE,SHOOTER_LEFT,SHOOTER_RIGHT);

	drivetrain->SetDeadband(0.25,0.25);

	lw = LiveWindow::GetInstance();

    // Show what command your subsystem is running on the SmartDashboard
    SmartDashboard::PutData(drivetrain);
}

Robot::~Robot(){
	delete drivetrain;
	delete holder;

}
void Robot::AutonomousInit() {
	std::cout << "Starting Auto" << std::endl;
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TeleopInit() {
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// this line or comment it out.
	std::cout << "Starting Teleop" << std::endl;
}

void Robot::TeleopPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {
	lw->Run();
}

START_ROBOT_CLASS(Robot);
