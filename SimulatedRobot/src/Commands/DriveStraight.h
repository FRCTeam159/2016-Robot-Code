/*
 * DriveStraight.h
 *
 *  Created on: Mar 3, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_DRIVESTRAIGHT_H_
#define SRC_COMMANDS_DRIVESTRAIGHT_H_
#include "Subsystems/GPMotor.h"
#include <Commands/Command.h>

#ifdef SIMULATION
#define PIDController MyPIDController
#endif
class DriveStraight: public Command {
	PIDController* pid;
	double distance;

public:
	DriveStraight(double distance);
	void Initialize();
	bool IsFinished();
	void Execute();
	void End();
	void Interrupted() { End();}
};

class DriveStraightPIDSource: public PIDSource {
public:
	virtual ~DriveStraightPIDSource();
	double PIDGet();
};

class DriveStraightPIDOutput: public PIDOutput {
public:
	virtual ~DriveStraightPIDOutput();
	void PIDWrite(float d);
};

#endif /* SRC_COMMANDS_DRIVESTRAIGHT_H_ */
