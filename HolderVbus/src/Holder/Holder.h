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
	GO_TO_REVERSE_LIMIT,
	BALL_PUSH_ERR,
};


class Holder {
	double gateTicksPerRevolution;
	double ballDetectionDelay;
	double pushTimeout;
	struct timeb start_time;
	struct timeb end_time;
	bool atReverseLimit;
	bool atForwardLimit;
	int pushComplete;
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


	DigitalInput IRsensor;

	void FindZero();
	void WaitForBallToEnter();
	void SetGateToForwardLimit();
	void WaitForPushRequest();
	void WaitForBallToLeave();
	void SetGateToReverseLimit();
	void RemoveBall();
	void SetPushMotorSpeed(double);
	int deltaTime(struct timeb* first, struct timeb* after);
public:
	enum push_states {
		PUSH_NOT_COMPLETE = 0,
		PUSH_COMPLETE = 1,
		PUSH_ERROR = 2,
	};
	Holder(int mtr1,int mtr2,int IR);
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
	int CheckPushed();
	bool IsLoaded();
	bool IsAtForwardLimit();
	bool IsAtReverseLimit();
};

#endif /* SRC_HOLDER_H_ */
