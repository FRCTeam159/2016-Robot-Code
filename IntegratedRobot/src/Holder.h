/*
 * Holder.h
 *
 *  Created on: Feb 1, 2016
 *      Author: alpin
 */

#ifndef SRC_HOLDER_H_
#define SRC_HOLDER_H_
class Victor;
class Holder {
private:
	Victor *gate;
	/* LimitSwitch *open
	 LimitSwitch *closed */

public:
	Holder();
	virtual ~Holder();
	bool CheckLoaded();//TODO
	bool GrabBall();//TODO
};

#endif /* SRC_HOLDER_H_ */
