/*
 * VerticalTargeter.h
 *
 *  Created on: Feb 5, 2016
 *      Author: alpin
 */

#ifndef SRC_VERTICALTARGETER_H_
#define SRC_VERTICALTARGETER_H_
class Launcher;
class Lidar;
class VerticalTargeter {
private:
	Launcher *siege;
	Lidar *laser;
public:
	VerticalTargeter(Launcher*, Lidar*);
	void Obey();
	void SetAngle(float);
	void SetSpeed(float);
	bool AngleGood(float);
	bool SpeedGood(float);
	int GetRange();//warning: takes a while
	virtual ~VerticalTargeter();
};

#endif /* SRC_VERTICALTARGETER_H_ */
