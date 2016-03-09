
#include <math.h>

#include "DriveTrain.h"
#include "Assignments.h"

#include "Commands/TankDriveWithJoystick.h"

DriveTrain::DriveTrain() : Subsystem("DriveTrain"),
	left_motor(DRIVE_LEFT),right_motor(DRIVE_RIGHT)
{
	std::cout<<"New DriveTrain("<<DRIVE_LEFT<<","<<DRIVE_RIGHT<<")"<<std::endl;
	SetDistancePerPulse(WHEEL_DIAMETER,WHEEL_TICKS,INVERT_RIGHT_SIDE);
	SetDeadband(DEADBAND,DEADBAND);
	disabled=true;

	// Let's show everything on the LiveWindow
	// TODO: LiveWindow::GetInstance()->AddActuator("Drive Train", "Front_Left Motor", (Talon) front_left_motor);
	// TODO: LiveWindow::GetInstance()->AddActuator("Drive Train", "Back Left Motor", (Talon) back_left_motor);
	// TODO: LiveWindow::GetInstance()->AddActuator("Drive Train", "Front Right Motor", (Talon) front_right_motor);
	// TODO: LiveWindow::GetInstance()->AddActuator("Drive Train", "Back Right Motor", (Talon) back_right_motor);
}

// ===========================================================================================================
// DriveTrain::SetDistancePerPulse(double d, double t, bool b)
// ===========================================================================================================
// inputs
//   d    wheel diameter (inches)
//   t    wheel encoder ticks per revolution
//   b    if true reverse encoder direction on right side
// notes:
//   right wheel encoder may need to be reversed but for some reason encoder->SetReverseDirection isn't
//   supported in simulation mode (throws exception) and can only be set in the constructor
//   (which is inconvenient when using "GPMotors", since that would require passing in an additional argument)
//   An alternative solution is to just invert the "DistancePerPulse" value
// ===========================================================================================================
void DriveTrain::SetDistancePerPulse(double d, double t, bool b){
	inverted =b;
	dpp=(double)M_PI*(d/12.0) / t;
	left_motor.SetDistancePerPulse(dpp);
	//right_motor.SetDistancePerPulse(b?-dpp:dpp);
	right_motor.SetDistancePerPulse(dpp);
	squared_inputs=false;
}
// ===========================================================================================================
// DriveTrain::GetDistancePerPulse()
//  - return Distance per encoder tick (in feet)
// ===========================================================================================================
double DriveTrain::GetDistancePerPulse() {
	return dpp;
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

void DriveTrain::Limit(double &num) {
  if (num > 1.0)
    num= 1.0;
  if (num < -1.0)
    num= -1.0;
}
// square the inputs (while preserving the sign) to increase fine control
// while permitting full power
void DriveTrain::SquareInputs(double &left, double &right) {
	if (left >= 0.0)
		left = (left * left);
	else
		left = -(left * left);
	if (right >= 0.0)
		right = (right * right);
	else
		right = -(right * right);
}

void DriveTrain::Drive(Joystick* joy) {
#if JOYTYPE == XBOX_GAMEPAD
	double left=-joy->GetRawAxis(1);
	double right=-joy->GetRawAxis(4);
#else
	double left=-joy->GetY();
	double right=-joy->GetRawAxis(4);
#endif
	Limit(left);
	Limit(right);
	if(squared_inputs)
		SquareInputs(left,right);
	left=Deadband(left,x_deadband);
	right=Deadband(right,y_deadband);
	//std::cout<<"left:"<<left<<" right:"<<right<<std::endl;
	left_motor.Set(left);
	if(inverted)
		right_motor.Set(-right);
	else
		right_motor.Set(right);
}

double DriveTrain::Deadband(double x, double ignore) {
	return fabs(x)>=ignore ? x: 0.0;
}
void DriveTrain::SetDeadband(double x, double y) {
	x_deadband=x;
	y_deadband=y;
}

void DriveTrain::SetPID(int mode, double P, double I, double D){
	left_motor.SetPID(mode,P,I,D);
	right_motor.SetPID(mode,P,I,D);
}
#define P 0.15
#define I 0.03
#define D 0.01
void DriveTrain::SetDistance(double d){
	std::cout << "DriveTrain::SetDistance:"<<d<<std::endl;
	right_motor.SetPID(GPMotor::POSITION,P,I, D);
	left_motor.SetPID(GPMotor::POSITION,P,I, D);
	//left_motor.SetDebug(1);
	//right_motor.SetDebug(1);


	left_motor.SetDistance(d);
	right_motor.SetDistance(d);
	left_motor.SetTolerance(0.1);
	right_motor.SetTolerance(0.1);
	left_motor.Enable();
	right_motor.Enable();

}
void DriveTrain::SetSpeed(double d){
	left_motor.SetVelocity(d);
	right_motor.SetVelocity(d);
}
void DriveTrain::Reset() {
	right_motor.Reset();
	left_motor.Reset();
}
void DriveTrain::Enable() {
	right_motor.Enable();
	left_motor.Enable();
	disabled=false;
}
void DriveTrain::Disable() {
	//left_motor.ClearPID();
	//right_motor.ClearPID();
	right_motor.Disable();
	left_motor.Disable();
	disabled=true;
}
bool DriveTrain::IsDisabled(){
	return disabled;
}

double DriveTrain::GetHeading() {
	return 0;//gyro->GetAngle();
}

void DriveTrain::TeleopInit() {
	std::cout << "DriveTrain::TeleopInit"<<std::endl;
	//gyro->Reset();
	left_motor.SetMode(GPMotor::VOLTAGE);
	right_motor.SetMode(GPMotor::VOLTAGE);
	Reset();
	Enable();
}

void DriveTrain::AutonomousInit() {
	std::cout << "DriveTrain::AutonomousInit"<<std::endl;
//	left_motor.SetPID(GPMotor::POSITION,0.2,0.001,1);
//	right_motor.SetPID(GPMotor::POSITION,0.2,0.001,1);
//	left_motor.SetDebug(1);
//	right_motor.SetDebug(1);
	Reset();
}

void DriveTrain::DisabledInit() {
	std::cout << "DriveTrain::DisabledInit"<<std::endl;
	left_motor.SetDebug(0);
	right_motor.SetDebug(0);
	Disable();
	right_motor.Reset();
	left_motor.Reset();
	//left_motor.ClearPID();
	//right_motor.ClearPID();
}

bool DriveTrain::OnTarget(){
	bool b1=left_motor.OnTarget();
	bool b2=right_motor.OnTarget();
	return b1 && b2;
}

double DriveTrain::GetDistance() {
	return (left_motor.GetDistance() + right_motor.GetDistance())/2;
}

double DriveTrain::GetLeftDistance(){
	return left_motor.GetDistance();

}
double DriveTrain::GetRightDistance(){
	return right_motor.GetDistance();
}
