#ifndef DriveTrain_H
#define DriveTrain_H

#include "WPILib.h"
#include "Subsystems/GPMotor.h"

/**
 * The DriveTrain subsystem incorporates the sensors and actuators attached to
 * the robots chassis. These include four drive motors, a left and right encoder
 * and a gyro.
 */
class DriveTrain : public Subsystem {
	GPMotor left_motor;
	GPMotor right_motor;
private:
	//AnalogGyro* gyro;
	double x_deadband,y_deadband;
	double dpp;
	bool inverted;
	bool squared_inputs;
	void Limit(double &num);
	void SquareInputs(double &left, double &right);
public:
	DriveTrain();

	void InitDefaultCommand();
	void SetSquaredInputs(double d){ squared_inputs=d;}
	void Log();
	void Reset();

	void Drive(Joystick* joy);
	double GetHeading();

	void TeleopInit();
	void AutonomousInit();
	void DisabledInit();

	double GetDistance();
	double GetLeftDistance();
	double GetRightDistance();

	virtual double Deadband(double x, double ignore);
	virtual void SetDeadband(double x, double y);
	virtual void SetDistancePerPulse(double diam,double ticks, bool b);
	virtual double GetDistancePerPulse();
	virtual bool IsInverted() { return inverted;}
	virtual void SetPID(int mode, double P, double I, double D);
	virtual void SetDistance(double d);
	virtual void SetSpeed(double d);
	virtual bool OnTarget();
};

#endif
