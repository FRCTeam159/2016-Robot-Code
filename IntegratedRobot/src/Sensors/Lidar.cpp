/*
 * Lidar.cpp
 *
 *  Created on: Feb 3, 2016
 *      Author: alpin
 */

#include <Sensors/Lidar.h>
#include <WPILib.h>
Lidar::Lidar(Port a, uint8_t b):I2C(a,b){
}

Lidar::~Lidar() {
	// TODO Auto-generated destructor stub
}


int Lidar::GetDistance()
{
	while(Write(0, 0x04))
	{
		Wait(.002);
	}
	uint8_t distanceAddress=0x8f;
	uint16_t distance;
	uint8_t distArr[2];
	Wait(.02);
	bool failure=false;
	while(!failure){
		if(!WriteBulk(&distanceAddress,1))
		{
			Wait(0.002);
			failure=!ReadOnly(2, (uint8_t*)distArr);
			distance = (distArr[0] * 256) + distArr[1];
		}
		Wait(.002);
	}
	return((int) distance);
}
