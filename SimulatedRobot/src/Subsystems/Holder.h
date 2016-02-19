/*
 * Holder.h
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#ifndef SRC_SUBSYSTEMS_HOLDER_H_
#define SRC_SUBSYSTEMS_HOLDER_H_
#include "WPILib.h"
#include <Commands/Subsystem.h>

class BallHolder: public Subsystem {
	Victor gateMotor;
	Victor pushMotor;
	bool gateopen;
public:
	BallHolder(int m1, int m2);
	bool GateIsOpen();
	void OpenGate();
	void CloseGate();
	void PushBall(bool);
};

#endif /* SRC_SUBSYSTEMS_HOLDER_H_ */
