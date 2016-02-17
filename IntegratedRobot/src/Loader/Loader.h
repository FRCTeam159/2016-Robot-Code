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
#include <Sensors/AngleAccelerometer.h>

class Loader {
private:
	//PIDController *liftMotor;//has an accelerometer, and a single limit switch for zeroing
	//Victor *rollerMotor;
	PIDController *sAngCtrl;
	AngleAccelerometer accel;
	void SetAngle(float);
	void SpinRoller(bool);
	int state;
	enum {
		SETLOWPOSITON,
		GRABBALL,
	};

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
	void setLowPosition();
	void grabBall();
public:
	Loader(int motor1, int motor2, I2C::Port p);
	void StartRoller();
	void StopRoller();
	void LowerLifter();
	void RaiseLifter();
	bool AngleGood(float);
	void Obey();
	virtual ~Loader();
	void TeleopInit();
	void TeleopPeriodic();
	void AutonomousInit();
	void AutonomousPeriodic();
};

#endif /* SRC_LOADER_H_ */
