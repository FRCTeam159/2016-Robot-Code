/*
 * Holder.h
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#ifndef SRC_HOLDER_H_
#define SRC_HOLDER_H_
#define CANTALON_PUSHER
#define CANTALON_GATE
#define GATEMOTOR_GR 26.9
#define GATEMOTOR_ET 7
#include <WPILib.h>


class Holder {
	double gateTicksPerRevolution;
	DigitalInput IRsensor;
	bool isAtReverseLimit;
	bool isAtForwardLimit;
#ifdef CANTALON_PUSHER
	CANTalon pushMotor;
#else
	Victor pushMotor;
#endif

#ifdef CANTALON_GATE
	CANTalon gateMotor;
	int readyToFire;
	int readyToLoad;
#else
	Victor gateMotor;
	DigitalInput readyToFire;
	DigitalInput readyToLoad;
#endif


	/* LimitSwitch *open
	 LimitSwitch *closed */
	void GrabBall();//TODO
	void PushBall();
	void LoadBall();
	void FindZero();
public:
	Holder(int mtr1,int mtr2,int ls1, int ls2,int IR);
	void AutoHold();
	bool IsLoaded();
	void Init();
};

#endif /* SRC_HOLDER_H_ */
