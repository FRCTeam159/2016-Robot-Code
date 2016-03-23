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

class Loader: public Subsystem, public PIDSource {
	GPMotor liftMotor;
	GPMotor rollerMotor;
	AnalogGyro accel;
	DigitalInput lowerLimit;

	double angle=0;
	double max_angle=80;
	double min_angle=0;
	bool rollers_on=false;
	bool at_limit=false;
	bool initialized;
	void Init();
	void Log();
	void Disable();

	double PIDGet();
	void InitDefaultCommand();
public:
	Loader();
	void SetTargetAngle(double a);
	double GetTargetAngle();
	bool AtAngle();
	bool AtLowerLimit();

	void TurnRollerOn(bool b);
	bool AreRollersOn();

	void AutonomousInit();
	void TeleopInit();
	void DisabledInit();

	void GoToLowerLimitSwitch();
	void SetLow();
	void SetMed();
	void SetHigh();
	void SetMax();
	bool IsInitialized();
	void SetInitialized();
	void Initialize();
};

#endif /* SRC_SUBSYSTEMS_LOADER_H_ */
