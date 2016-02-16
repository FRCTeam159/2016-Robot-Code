/*
 * Loader.h
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#ifndef SRC_LOADER_H_
#define SRC_LOADER_H_
class PIDController;
class Victor;
class Loader {
private:
	PIDController *liftMotor;//has an accelerometer, and a single limit switch for zeroing
	Victor *rollerMotor;
	void SetAngle(float);
	void SpinRoller(float);
public:
	Loader(PIDController*, Victor*);
	void StartRoller();
	void StopRoller();
	void LowerLifter();
	void RaiseLifter();
	bool AngleGood(float);
	void Obey();
	virtual ~Loader();
};

#endif /* SRC_LOADER_H_ */
