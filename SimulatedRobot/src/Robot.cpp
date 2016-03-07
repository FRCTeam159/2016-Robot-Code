
#include "Robot.h"
#include "Assignments.h"

std::shared_ptr<DriveTrain> Robot::drivetrain;
std::shared_ptr<BallHolder> Robot::holder;
std::shared_ptr<Shooter> Robot::shooter;
std::unique_ptr<OI> Robot::oi;


void Robot::RobotInit() {
	std::cout << "Robot::RobotInit" << std::endl;
	drivetrain.reset(new DriveTrain());
	holder.reset(new BallHolder());
	shooter.reset(new Shooter());
	oi.reset(new OI());

	SmartDashboard::PutData(drivetrain.get());
	SmartDashboard::PutData(holder.get());
	SmartDashboard::PutData(shooter.get());
}

void Robot::AutonomousInit() {
	std::cout << "Robot::AutonomousInit" << std::endl;
	drivetrain->AutonomousInit();
	shooter->AutonomousInit();
	holder->AutonomousInit();

	//autonomousCommand.Cancel();
	autonomousCommand.Start();
}

void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
}

void Robot::DisabledInit(){
	std::cout << "Robot::DisabledInit" << std::endl;
	drivetrain->DisabledInit();
	shooter->DisabledInit();
	holder->DisabledInit();
}
void Robot::DisabledPeriodic(){
	//shooter->Disable();
}

void Robot::TeleopInit() {
	std::cout << "Robot::TeleopInit" << std::endl;
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// this line or comment it out.
	autonomousCommand.Cancel();
	shooter->TeleopInit();
	holder->TeleopInit();
	drivetrain->TeleopInit();
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
