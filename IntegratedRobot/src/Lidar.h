/*
 * Lidar.h
 *
 *  Created on: Feb 3, 2016
 *      Author: alpin
 */

#ifndef SRC_LIDAR_H_
#define SRC_LIDAR_H_
#include <WPILib.h>
class Lidar: public I2C {
public:
	Lidar(Port, uint8_t);
	virtual ~Lidar();
	int GetDistance();//returns cm
};

#endif /* SRC_LIDAR_H_ */
