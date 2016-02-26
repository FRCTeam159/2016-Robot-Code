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
	double angle;
public:
	Shooter(int,int,int);
	void Shoot(bool);
	void SetTargetAngle(double a);
	double GetTargetAngle();
	bool OnTarget();
	void Init();
	void Disable();
};

#endif /* SRC_SUBSYSTEMS_SHOOTER_H_ */
