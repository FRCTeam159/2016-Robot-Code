/*
 * SRXSlave.h
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#ifndef SRC_SRXCONFIGS_SRXSLAVE_H_
#define SRC_SRXCONFIGS_SRXSLAVE_H_
#include <WPILib.h>
class SRXSlave: public CANTalon {
private:
	int master;
public:
	SRXSlave(int id, int masterid);
	virtual ~SRXSlave();
	void Obey();
};

#endif /* SRC_SRXCONFIGS_SRXSLAVE_H_ */
