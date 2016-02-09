/*
 * SRXSpeed.h
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#ifndef SRC_SRXCONFIGS_SRXSPEED_H_
#define SRC_SRXCONFIGS_SRXSPEED_H_
#include <WPILib.h>
class SRXSpeed: public CANTalon{
private:
	int maxTicks=1;
	float target=0;
public:
	SRXSpeed(int id, double Pvalue, double Ivalue, double Dvalue, int);
	void SetTargetSpeed(float);
	void Obey();
	void ChangePID(float, float, float);
	bool CloseEnough(float);
	virtual ~SRXSpeed();
};

#endif /* SRC_SRXCONFIGS_SRXSPEED_H_ */
