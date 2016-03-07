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
	double angle,max_angle,min_angle;
	void Init();
	void Disable();

public:
	Shooter();
	void Shoot(bool);
	void SetTargetAngle(double a);
	double GetTargetAngle();
	double GetMaxAngle() { return max_angle;}
	double GetMinAngle() { return min_angle;}

	bool OnTarget();
	void AutonomousInit();
	void TeleopInit();
	void DisabledInit();

};

#endif /* SRC_SUBSYSTEMS_SHOOTER_H_ */
