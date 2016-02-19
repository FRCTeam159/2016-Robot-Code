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
	int iterations=0;
	bool failed=false;
	while(Write(0, 0x04)&&!failed)
	{
		Wait(.002);
		iterations++;
		if(iterations>3)
		{
			failed=true;
		}
	}
	uint16_t distance;
	if(!failed)
	{
		iterations = 0;
		uint8_t distanceAddress=0x8f;
		uint8_t distArr[2];
		Wait(.02);
		bool finished=false;
		while(!finished&&!failed){
			if(!WriteBulk(&distanceAddress,1))
			{
				finished=!ReadOnly(2, (uint8_t*)distArr);
				distance = (distArr[0] * 256) + distArr[1];
				iterations++;
				if(iterations>3)
				{
					failed=true;
				}
			}
			Wait(.002);
		}
	}
	if(failed)
	{
		std::cout<<"Lidar read failed!!!"<<std::endl;
		distance=1234;//1234 is an error
	}
	return((int) distance);
}
