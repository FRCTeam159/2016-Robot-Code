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
	bool disabled;
	bool pid_disabled;

	void DisablePID();
	void EnablePID();
	void Limit(double &num);
	void SquareInputs(double &left, double &right);
	void SetDistance(double d);

public:
	DriveTrain();

	void InitDefaultCommand();
	void SetSquaredInputs(double d){ squared_inputs=d;}
	void Log();
	void Reset();
	void Enable();
	void Disable();
	void EndTravel();

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
	virtual void SetPID(int mode, double P, double I, double D);
	virtual void DriveStraight(double d);
	virtual void SetSpeed(double d);
	virtual bool OnTarget();
	virtual void SetInverted(bool b) { inverted=b;}

	virtual bool IsDisabled() { return disabled;}
	virtual bool IsPIDDisabled() { return pid_disabled;}
	virtual bool IsInverted() { return inverted;}

};

#endif
