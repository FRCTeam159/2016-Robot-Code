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
#include <time.h>
#include <sys/timeb.h>
enum {
	FIND_ZERO,
	WAIT_FOR_BALL_TO_ENTER,
	GO_TO_FORWARD_LIMIT,
	WAIT_FOR_PUSH_REQUEST,
	WAIT_FOR_BALL_TO_LEAVE,
	GO_TO_REVERSE_LIMIT
};


class Holder {
	double gateTicksPerRevolution;
	double ballDetectionDelay;
	double pushTimeout;
	struct timeb start_time;
	struct timeb end_time;
	bool atReverseLimit;
	bool atForwardLimit;
	bool pushComplete;
	bool pushRequested;
	bool foundZero;
	int state;
#ifdef CANTALON_GATE
	CANTalon gateMotor;
#else
	Victor gateMotor;
#endif

#ifdef CANTALON_PUSHER
	CANTalon pushMotor;
#else
	Victor pushMotor;
#endif

#ifdef CANTALON_GATE
	int revGateLimit;
	int fwdGateLimit;
#else
	DigitalInput revGateLimit;
	DigitalInput fwdGateLimit;
#endif

	DigitalInput IRsensor;

	void FindZero();
	void WaitForBallToEnter();
	void SetGateToForwardLimit();
	void WaitForPushRequest();
	void WaitForBallToLeave();
	void SetGateToReverseLimit();
	void SetPushMotorSpeed(double);
	int deltaTime(struct timeb* first, struct timeb* after);
public:
	Holder(int mtr1,int mtr2,int ls1, int ls2,int IR);
	void PushBall();
	void AutoHold();
	void Init();
	void Test();
	void TestInit();
	void TeleopInit();
	void Disable();
	void TestPeriodic();
	void TeleopPeriodic();
	void AutonomousInit();
	void AutonomousPeriodic();
	bool CheckPushed();
	bool IsLoaded();
	bool IsAtForwardLimit();
	bool IsAtReverseLimit();
};

#endif /* SRC_HOLDER_H_ */
