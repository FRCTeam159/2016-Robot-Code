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
public:
	BallHolder(int m1, int m2);
};

#endif /* SRC_SUBSYSTEMS_HOLDER_H_ */
