/*
 * Loader.h
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#ifndef SRC_LOADER_H_
#define SRC_LOADER_H_
#define CANTALON_LIFTMOTOR
#define CANTALON_ROLLERMOTOR
#include <WPILib.h>
#include <Sensors/IntakeAngleAccelerometer.h>
#include <time.h>
#include <sys/timeb.h>

class Loader {
private:
	//PIDController *liftMotor;//has an accelerometer, and a single limit switch for zeroing
	//Victor *rollerMotor;
	PIDController *sAngCtrl;
	void SetAngle(float);
	void SpinRollers(bool);
	int state;
	bool atLimit;
	double targetAngle;
	int oldState;
	double timeoutTime;
	float ManualTarget= 0;


#ifdef CANTALON_LIFTMOTOR
	CANTalon liftMotor;
#else
	Victor liftMotor;
#endif

#ifdef CANTALON_ROLLERMOTOR
	CANTalon rollerMotor;
#else
	Victor rollerMotor;
#endif
	IntakeAngleAccelerometer accel;
	void GoToZeroLimitSwitch();
	bool IsAtLimit();
	void StopRollers();
public:
	struct timeb start_time;
	struct timeb end_time;
	enum {
		LOW,
		HIGH,
		MED,
		CANCEL,
		MANUAL,
	};
	Loader(int motor1, int motor2, I2C::Port p);
	bool AtGrabAngle();
	bool AtZeroAngle();
	void LowerLifter();
	void RaiseLifter();
	bool AngleGood(float);
	void Obey();
	virtual ~Loader();
	void TeleopInit();
	void TeleopPeriodic();
	void AutonomousInit();
	void AutonomousPeriodic();
	void DisabledInit();
	void SetLow();
	void SetMed();
	void SetHigh();
	void SetManual();
	void SetManualPower(float);
	void SetTimeoutTime(double t){
		timeoutTime = t;
	}
	void Continue();
	void Cancel();
	void ExpelBall();
	int deltaTime(struct timeb* first, struct timeb* after);
	int GetState(){
		return state;
	}
};

#endif /* SRC_LOADER_H_ */
