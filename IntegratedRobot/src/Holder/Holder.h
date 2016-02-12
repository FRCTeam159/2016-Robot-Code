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
#define GATEMOTOR_GR 71
#define GATEMOTOR_ET 7
#include <WPILib.h>
enum {
	FIND_ZERO,
	WAIT_FOR_BALL_TO_ENTER,
	GO_TO_FORWARD_LIMIT,
	WAIT_FOR_BALL_TO_LEAVE,
	GO_TO_REVERSE_LIMIT
};


class Holder {
	double gateTicksPerRevolution;
	DigitalInput IRsensor;
	bool atReverseLimit;
	bool atForwardLimit;
	int state;
	double ballDetectionDelay;
#ifdef CANTALON_PUSHER
	CANTalon pushMotor;
#else
	Victor pushMotor;
#endif

#ifdef CANTALON_GATE
	CANTalon gateMotor;
	int revGateLimit;
	int fwdGateLimit;
#else
	Victor gateMotor;
	DigitalInput revGateLimit;
	DigitalInput fwdGateLimit;
#endif


	/* LimitSwitch *open
	 LimitSwitch *closed */
	void GrabBall();//TODO
	void PushBall();
	void LoadBall();
	void FindZero();
public:
	Holder(int mtr1,int mtr2,int ls1, int ls2,int IR);
	bool isAtReverseLimit();
	bool isAtForwardLimit();
	void AutoHold();
	bool IsLoaded();
	void WaitForBallToEnter();
	void WaitForBallToLeave();
	void SetGateToForwardLimit();
	void SetGateToReverseLimit();
	void Init();
	void Test();
	void TestInit();
	void TeleopInit();
	void Disable();
	void TestPeriodic();
	void TeleopPeriodic();
};

#endif /* SRC_HOLDER_H_ */
