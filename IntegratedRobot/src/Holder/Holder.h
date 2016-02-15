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
	DigitalInput IRsensor;
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

	void LoadBall();
	void FindZero();
	void WaitForBallToEnter();
	void WaitForBallToLeave();
	void WaitForPushRequest();
	void SetGateToForwardLimit();
	void SetGateToReverseLimit();
	void SetPushMotorSpeed(double);
	int deltaTime(struct timeb* first, struct timeb* after);
public:
	Holder(int mtr1,int mtr2,int ls1, int ls2,int IR);
	void PushBall();
	bool IsAtReverseLimit();
	bool IsAtForwardLimit();
	void AutoHold();
	bool IsLoaded();
	void Init();
	void Test();
	void TestInit();
	void TeleopInit();
	void Disable();
	void TestPeriodic();
	void TeleopPeriodic();
	bool CheckPushed();
	void AutonomousInit();
	void AutonomousPeriodic();
};

#endif /* SRC_HOLDER_H_ */
