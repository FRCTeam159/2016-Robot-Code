/*
 * Holder.h
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#ifndef SRC_SUBSYSTEMS_BALLHOLDER_H_
#define SRC_SUBSYSTEMS_BALLHOLDER_H_
#include "WPILib.h"
#include "Subsystems/GPMotor.h"

class BallHolder: public Subsystem {
	GPMotor gateMotor;
	GPMotor pushMotor;
	DigitalInput lowerLimit;
	DigitalInput upperLimit;
	AnalogInput ballSensor;

	bool gateopen;
	void Init();

public:
	BallHolder();
	bool IsGateOpen();
	bool IsGateClosed();
	bool IsBallPresent();
	void OpenGate();
	void CloseGate();
	void PushBall(bool);

	void AutonomousInit();
	void TeleopInit();
	void DisabledInit();

};

#endif /* SRC_SUBSYSTEMS_BALLHOLDER_H_ */
