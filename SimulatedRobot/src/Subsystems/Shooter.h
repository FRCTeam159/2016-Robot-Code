/*
 * Shooter.h
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#ifndef SRC_SUBSYSTEMS_SHOOTER_H_
#define SRC_SUBSYSTEMS_SHOOTER_H_

#include "WPILib.h"
#include <Commands/Subsystem.h>

class Shooter: public Subsystem {
	Victor angleMotor;
	Victor leftMotor;
	Victor rightMotor;
public:
	Shooter(int,int,int);
};

#endif /* SRC_SUBSYSTEMS_SHOOTER_H_ */
