/*
 * AngleAdjuster.h
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#ifndef SRC_ANGLEADJUSTER_H_
#define SRC_ANGLEADJUSTER_H_
class SRXPosition;
class AngleAdjuster {
private:
	SRXPosition *motor;
	float targetAngle;
	float ticksPerDegree;
public:
	AngleAdjuster(SRXPosition*, float);//float is ticks/degree
	void SetAngle(float);//give this degrees
	void Obey();
	float GetCurrentAngle();
	bool CloseEnough(int);
	virtual ~AngleAdjuster();
};

#endif /* SRC_ANGLEADJUSTER_H_ */
