#include "DriveTrain.h"
#include "Commands/TankDriveWithJoystick.h"


#include <math.h>
DriveTrain::DriveTrain(int m1, int m2) : Subsystem("DriveTrain"),
	left_motor(m1),right_motor(m2)
{
	std::cout<<"New DriveTrain("<<m1<<","<<m2<<")"<<std::endl;
	// Encoders may measure differently in the real world and in
	// simulation. In this example the robot moves 0.042 barleycorns
	// per tick in the real world, but the simulated encoders
	// simulate 360 tick encoders. This if statement allows for the
	// real robot to handle this difference in devices.
#ifdef REAL
	double dpp=0.042;
#else
	// Circumference in ft = 7.5in/12(in/ft)*PI
	double dpp=(double) (7.5/12.0*M_PI) / 360.0;
#endif
	drive = new RobotDrive(&left_motor, &right_motor);
	left_motor.SetDistancePerPulse(dpp);
	right_motor.SetDistancePerPulse(dpp);

	//drive->SetInvertedMotor(RobotDrive::kFrontRightMotor, false);	// invert the left side motors

	//rangefinder = new AnalogInput(6);
	//gyro = new AnalogGyro(1);

	// Let's show everything on the LiveWindow
	// TODO: LiveWindow::GetInstance()->AddActuator("Drive Train", "Front_Left Motor", (Talon) front_left_motor);
	// TODO: LiveWindow::GetInstance()->AddActuator("Drive Train", "Back Left Motor", (Talon) back_left_motor);
	// TODO: LiveWindow::GetInstance()->AddActuator("Drive Train", "Front Right Motor", (Talon) front_right_motor);
	// TODO: LiveWindow::GetInstance()->AddActuator("Drive Train", "Back Right Motor", (Talon) back_right_motor);
	// LiveWindow::GetInstance()->AddSensor("Drive Train", "Left Encoder", left_encoder);
	// LiveWindow::GetInstance()->AddSensor("Drive Train", "Right Encoder", right_encoder);
	// LiveWindow::GetInstance()->AddSensor("Drive Train", "Rangefinder", rangefinder);
	// LiveWindow::GetInstance()->AddSensor("Drive Train", "Gyro", gyro);
}

/**
 * When no other command is running let the operator drive around
 * using the PS3 joystick.
 */
void DriveTrain::InitDefaultCommand() {
	SetDefaultCommand(new TankDriveWithJoystick());
}

/**
 * The log method puts interesting information to the SmartDashboard.
 */
void DriveTrain::Log() {
	SmartDashboard::PutNumber("Left Distance", left_motor.GetDistance());
	SmartDashboard::PutNumber("Right Distance", right_motor.GetDistance());
	SmartDashboard::PutNumber("Left Speed", left_motor.GetVelocity());
	SmartDashboard::PutNumber("Right Speed", right_motor.GetVelocity());
	//SmartDashboard::PutNumber("Gyro", gyro->GetAngle());
}

void DriveTrain::Drive(double left, double right) {
	drive->TankDrive(left, right,true);
}

void DriveTrain::Drive(Joystick* joy) {
	//Drive(-joy->GetY(), -joy->GetRawAxis(4));
	double left=-joy->GetRawAxis(1);
	double right=-joy->GetRawAxis(4);
	left=Deadband(left,x_deadband);
	right=Deadband(right,y_deadband);
	//std::cout<<"left:"<<left<<" right:"<<right<<std::endl;
	Drive(left, right);
}

double DriveTrain::Deadband(double x, double ignore) {
	return fabs(x)>=ignore ? x: 0.0;
}
void DriveTrain::SetDeadband(double x, double y) {
	x_deadband=x;y_deadband=y;
}

double DriveTrain::GetHeading() {
	return 0;//gyro->GetAngle();
}

void DriveTrain::Reset() {
	//gyro->Reset();
	right_motor.Reset();
	left_motor.Reset();
}

double DriveTrain::GetDistance() {
	return (left_motor.GetDistance() + right_motor.GetDistance())/2;
}

double DriveTrain::GetDistanceToObstacle() {
	// Really meters in simulation since it's a rangefinder...
	return 0;//rangefinder->GetAverageVoltage();
}
