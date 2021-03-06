/*
 * Launcher.h
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#ifndef SRC_LAUNCHER_H_
#define SRC_LAUNCHER_H_
class SRXSpeed;
class PIDController;
class Launcher {
private:
	SRXSpeed *left, *right;
	PIDController *pid;
	float targetSpeed=0;
	float targetAngle=0;
public:
	Launcher(SRXSpeed*, SRXSpeed*, PIDController*);
	void SetTargetSpeed(float);
	void SetAngle(float);
	void Obey();
	bool AngleGood(float);
	bool SpeedGood(float);
	void Aim(float);
	virtual ~Launcher();
};

#endif /* SRC_LAUNCHER_H_ */
