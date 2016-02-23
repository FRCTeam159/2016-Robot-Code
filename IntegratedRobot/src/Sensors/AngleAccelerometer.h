/*
 * AngleAccelerometer.h
 *
 *  Created on: Feb 9, 2016
 *      Author: alpin
 */

#ifndef SRC_SHOOTANGLEACCELEROMETER_H_
#define SRC_SHOOTANGLEACCELEROMETER_H_
#include <WPILib.h>
class ShootAngleAccelerometer:public PIDSource {
private:
	Accelerometer *accel;  // ADXL345 digital accelerometer
	double analogAccelX;         // variable with smoothed acceleration value
	double analogAccelY;         // variable with smoothed acceleration value
	double analogAccelZ;         // variable with smoothed acceleration value
	double weight = 0.20;
	double initialX = 0.0;  // values measured when the device was 'flat'
	double initialY = 0.0;
	double initialZ = 0.0;
	double accelerationX;
	double accelerationY;
	double accelerationZ;
public:
	void SetPIDSourceType(PIDSourceType);
	double PIDGet();
	ShootAngleAccelerometer(I2C::Port);
	virtual ~ShootAngleAccelerometer();
};

#endif /* SRC_ANGLEACCELEROMETER_H_ */
