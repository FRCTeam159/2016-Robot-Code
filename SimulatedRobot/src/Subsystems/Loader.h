/*
 * Loader.h
 *
 *  Created on: Mar 21, 2016
 *      Author: alpiner
 */

#ifndef SRC_SUBSYSTEMS_LOADER_H_
#define SRC_SUBSYSTEMS_LOADER_H_

//#include <Commands/Subsystem.h>
#include "WPILib.h"
#include "Subsystems/GPMotor.h"

class Loader: public Subsystem {
	GPMotor angleMotor;
	GPMotor rollerMotor;
	AnalogGyro angleGyro;
	double angle=0;
	double max_angle=80;
	double min_angle=0;
	bool rollers_on=false;
	void Init();
	void Disable();
	void Log();

public:
	Loader();
	void SetTargetAngle(double a);
	double GetTargetAngle();
	bool IsAtAngle();
	void TurnRollerOn(bool b);
	bool AreRollersOn();

	void AutonomousInit();
	void TeleopInit();
	void DisabledInit();

};

#endif /* SRC_SUBSYSTEMS_LOADER_H_ */
