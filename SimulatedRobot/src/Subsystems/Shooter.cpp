/*
 * Shooter.cpp
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#include <Subsystems/Shooter.h>

Shooter::Shooter(int m1, int m2, int m3) : Subsystem("Shooter"),
	angleMotor(m1), leftMotor(m2),rightMotor(m3)
{
	// TODO Auto-generated constructor stub

}

