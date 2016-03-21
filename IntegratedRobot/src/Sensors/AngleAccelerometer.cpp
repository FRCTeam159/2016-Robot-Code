/*
 * AngleAccelerometer.cpp
 *
 *  Created on: Feb 9, 2016
 *      Author: alpin
 */

#include <Sensors/AngleAccelerometer.h>
#define PI 3.14159265
ShootAngleAccelerometer::ShootAngleAccelerometer(I2C::Port p) {
	accel = new ADXL345_I2C(p, Accelerometer::Range::kRange_4G, 0x53);
	m_pidSource = PIDSourceType::kDisplacement;
}

ShootAngleAccelerometer::~ShootAngleAccelerometer() {
	// TODO Auto-generated destructor stub

}

double ShootAngleAccelerometer::PIDGet()
{
	double dAngleX, dAngleY, dAngleZ;
	double x,y,z;
	double result;
	accelerationX = (accel->GetX() * weight) + (accelerationX * (1.0 - weight));
	accelerationY = (accel->GetY() * weight) + (accelerationY * (1.0 - weight));
	accelerationZ = (accel->GetZ() * weight) + (accelerationZ * (1.0 - weight));
	// printf("digital acceleration x=%f y=%f z=%f\n", accelerationX, accelerationY, accelerationZ);
	x = accelerationX * accelerationX;
	y = accelerationY * accelerationY;
	z = accelerationZ * accelerationZ;

	// X Axis
	result = accelerationX / sqrt(y+z);
	dAngleX = atan(result) * (180.0/PI);

	// Y Axis
	result = accelerationY / sqrt(x);
	dAngleY = atan(result) * (180.0/PI);
	dAngleY=90-dAngleY;

	// Z Axis
	result = accelerationZ / sqrt(x + y);
	dAngleZ = atan(result) * (180.0/PI);

	//		printf("digital angles x=%f, y=%f, z=%f analog x=%f y=%f z=%f\n",
	//				dAngleX, dAngleY, dAngleZ, angleX, angleY, angleZ);
//	printf("%f, %f, %f\n",
//	dAngleX, dAngleY, dAngleZ);
	float mountErr=4;
	return(dAngleY-mountErr);
}
void ShootAngleAccelerometer::SetPIDSourceType(PIDSourceType type)
{
	m_pidSource=type;
}
