/*
 * SRXSlave.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#include <SRXSlave.h>
#include <WPILib.h>
SRXSlave::SRXSlave(int id, int masterid):CANTalon(id) {
	// TODO Auto-generated constructor stub
	SetControlMode(CANTalon::kFollower);
	EnableControl();
	master=masterid;
}

SRXSlave::~SRXSlave() {
	// TODO Auto-generated destructor stub
}

void SRXSlave::Obey()
{
	Set(master);
}
