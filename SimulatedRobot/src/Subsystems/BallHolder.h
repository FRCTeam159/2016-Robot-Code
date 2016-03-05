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
public:
	BallHolder(int m1, int m2);
	void Init();
	bool IsGateOpen();
	bool IsGateClosed();
	bool IsBallPresent();
	void OpenGate();
	void CloseGate();
	void PushBall(bool);
};

#endif /* SRC_SUBSYSTEMS_BALLHOLDER_H_ */
