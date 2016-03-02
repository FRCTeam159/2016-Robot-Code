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
	bool gateopen;
public:
	BallHolder(int m1, int m2);
	void Init();
	bool GateIsOpen();
	void OpenGate();
	void CloseGate();
	void PushBall(bool);
};

#endif /* SRC_SUBSYSTEMS_BALLHOLDER_H_ */
