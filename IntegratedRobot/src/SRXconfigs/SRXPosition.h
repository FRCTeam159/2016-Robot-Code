/*
 * SRXPosition.h
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#ifndef SRC_SRXCONFIGS_SRXPOSITION_H_
#define SRC_SRXCONFIGS_SRXPOSITION_H_
#include<WPILib.h>
class SRXPosition: public CANTalon {
private:
	int target;
	bool invertMotor;
public:
	SRXPosition(int, double, double, double, bool);
	void ZeroSelf();
	virtual ~SRXPosition();
};

#endif /* SRC_SRXCONFIGS_SRXPOSITION_H_ */
