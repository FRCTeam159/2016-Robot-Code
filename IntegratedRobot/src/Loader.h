/*
 * Loader.h
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#ifndef SRC_LOADER_H_
#define SRC_LOADER_H_
class AngleAdjuster;
class Victor;
class Loader {
private:
	AngleAdjuster *liftMotor;//has an encoder, and a single limit switch for zeroing
	Victor *rollerMotor;
public:
	Loader(AngleAdjuster*, Victor*);
	bool SetAngle(float);
	void SpinRoller();
	virtual ~Loader();
};

#endif /* SRC_LOADER_H_ */
