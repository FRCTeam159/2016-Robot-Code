/*
 * Launcher.h
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#ifndef SRC_LAUNCHER_H_
#define SRC_LAUNCHER_H_
class SRXSpeed;
#include "AngleAccelerometer.h"
class PIDController;
class Launcher {
private:
	SRXSpeed *left, *right;
	AngleAccelerometer *angler;
	PIDController *pid;
	float targetSpeed=0;
	float targetAngle=0;
public:
	Launcher(SRXSpeed*, SRXSpeed*, AngleAccelerometer*, PIDController*);
	void SetTargetSpeed(float);
	void SetAngle(float);
	void Obey();
	bool AngleGood(float);
	bool SpeedGood(float);
	virtual ~Launcher();
};

#endif /* SRC_LAUNCHER_H_ */
