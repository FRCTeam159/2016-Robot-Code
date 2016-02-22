
#include "Robot.h"

std::shared_ptr<DriveTrain> Robot::drivetrain;
std::shared_ptr<BallHolder> Robot::holder;
std::shared_ptr<Shooter> Robot::shooter;
std::unique_ptr<OI> Robot::oi;

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
	drivetrain.reset(new DriveTrain(DRIVE_LEFT,DRIVE_RIGHT));
	holder.reset(new BallHolder(HOLDER_GATE,HOLDER_PUSH));
	shooter.reset(new Shooter(SHOOTER_ANGLE,SHOOTER_LEFT,SHOOTER_RIGHT));
	oi.reset(new OI());

	drivetrain->SetDeadband(0.25,0.25);

	SmartDashboard::PutData(drivetrain.get());
	SmartDashboard::PutData(holder.get());
	SmartDashboard::PutData(shooter.get());
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
	//autonomousCommand.Cancel();
	std::cout << "Starting Teleop" << std::endl;
}

void Robot::TeleopPeriodic() {
	// Scheduler runs "default" commands for all subsystems
	// e.g. void DriveTrain::InitDefaultCommand() {
	//	    SetDefaultCommand(new TankDriveWithJoystick()); }
	Scheduler::GetInstance()->Run();
}

void Robot::TestPeriodic() {
	lw->Run();
}

START_ROBOT_CLASS(Robot);
