/*
 * Particle.cpp
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#include <Particle.h>
#include <math.h>
Particle::Particle() {
	// TODO Auto-generated constructor stub

}

Particle::~Particle() {
	// TODO Auto-generated destructor stub
}

double Particle::AreaOverBoundingArea()
{
	double base = fabs(BoundingRectLeft-BoundingRectRight);
	double height = fabs(BoundingRectTop-BoundingRectBottom);
	double rectArea = base*height;
	return(areaPixels/rectArea);
}

double Particle::GetDistance()
{
	double height=fabs(BoundingRectTop-BoundingRectBottom)/1;//the one is a placeholder for the cosine of the angle
	//between the plane perpendicular to the camera and vertical. (the angle is 0 when the camera is horizontal)
	static double factor= 14*240.0/(0.3057*2);//14 is target height, 480 is screen height, .33459 is tan(18.5 deg)
	return(factor/height);
}

double Particle::DetermineScore(double target, double actual)
{
	double finalScore=((-100/target)*fabs(actual-target))+100.0;
	return(finalScore);
}
