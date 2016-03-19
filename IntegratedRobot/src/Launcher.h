/*
 * Launcher.h
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#ifndef SRC_LAUNCHER_H_
#define SRC_LAUNCHER_H_
class SRXSpeed;
class PIDSource;
class CANTalon;
class Launcher {
private:
	SRXSpeed *left, *right;
	PIDSource *pid;
	CANTalon *shootAngle;
	int cycle=1;
	float currentAngle;
	float targetSpeed=0;
	float targetAngle=0;
	bool atAngle = false;
	void ClumsyControl();
public:
	Launcher(SRXSpeed*, SRXSpeed*, PIDSource*, CANTalon*);
	void SetTargetSpeed(float);
	void SetAngle(float);
	void Obey();
	bool AngleGood(float);
	bool SpeedGood(float);
	void Aim(float);
	virtual ~Launcher();
};

#endif /* SRC_LAUNCHER_H_ */
