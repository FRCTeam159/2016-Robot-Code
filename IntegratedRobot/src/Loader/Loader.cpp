/*
 * Loader.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#include <Loader/Loader.h>
#include <WPILib.h>
#include <AngleAdjuster.h>
Loader::Loader(PIDController* a, Victor* b):liftMotor(a), rollerMotor(b) {
	//TODO Auto-generated contructtor stub
}

Loader::~Loader() {
	// TODO Auto-generated destructor stub
}
