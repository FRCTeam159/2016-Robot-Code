/*
 * Shooter.h
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#ifndef SRC_SUBSYSTEMS_SHOOTER_H_
#define SRC_SUBSYSTEMS_SHOOTER_H_

#include "WPILib.h"
#include "Subsystems/GPMotor.h"

class Shooter: public Subsystem {
	GPMotor angleMotor;
	GPMotor leftMotor;
	GPMotor rightMotor;
	AnalogGyro angleGyro;
	double angle,max_angle,min_angle;
	double flywheel_target;
	double flywheel_speed;
	void Init();

public:
	Shooter();
	void SetTargetAngle(double a);
	void SetTargetSpeed(double a);
	double GetTargetAngle();
	double GetMaxAngle() { return max_angle;}
	double GetMinAngle() { return min_angle;}
	void Disable();
	void DisableFlywheels();
	void EnableFlywheels();

	bool IsAtAngle();
	bool IsAtSpeed();
	double GetTargetSpeed();
	double GetSpeed();
	double GetAngle();

	void AutonomousInit();
	void TeleopInit();
	void DisabledInit();
	void Log();
	void LogSpeed(double d);
	void LogAngle(double d);


};

#endif /* SRC_SUBSYSTEMS_SHOOTER_H_ */
