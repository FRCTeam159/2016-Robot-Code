/*
 * Launcher.h
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#ifndef SRC_LAUNCHER_H_
#define SRC_LAUNCHER_H_
class SRXSpeed;
#include <AngleAdjuster.h>
class Launcher {
private:
	SRXSpeed *left, *right;
	AngleAdjuster *angler;
	float targetSpeed=0;
	float targetAngle=0;
public:
	Launcher(SRXSpeed*, SRXSpeed*, AngleAdjuster*);
	void SetTargetSpeed(float);
	void SetAngle(float);
	void Obey();
	bool AngleGood(float);
	bool SpeedGood(float);
	virtual ~Launcher();
};

#endif /* SRC_LAUNCHER_H_ */
