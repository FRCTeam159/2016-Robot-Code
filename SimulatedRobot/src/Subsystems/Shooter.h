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
	double min;
	double max;
public:
	Shooter(int,int,int);
	void Shoot();
	void SetTargetAngle(double a);
	double GetTargetAngle();
	double GetAngle();
	void SetLimits(double a1, double a2);
	void Init();
};

#endif /* SRC_SUBSYSTEMS_SHOOTER_H_ */
