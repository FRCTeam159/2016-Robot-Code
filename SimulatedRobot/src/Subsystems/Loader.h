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
	void Disable();

	double PIDGet();
	void InitDefaultCommand();
public:
	Loader();
	void Log();

	void SetLifterAngle(double a);
	double GetLifterAngle();
	bool LifterAtTargetAngle();
	bool LifterAtLowerLimit();
	bool LifterTestLowerLimit();

	void SpinRollers(bool b);
	void TurnRollersOff();
	bool RollersAreOn();

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
