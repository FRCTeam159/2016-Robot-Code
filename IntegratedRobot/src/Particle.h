/*
 * Particle.h
 *
 *  Created on: Jan 30, 2016
 *      Author: alpin
 */

#ifndef SRC_PARTICLE_H_
#define SRC_PARTICLE_H_

class Particle {
private:
	double areaScreen=0;
	double areaPixels;
	double aspect;
	double BoundingRectTop;
	double BoundingRectLeft;
	double BoundingRectRight;
	double BoundingRectBottom;
	double moment;
	double randomTest;
	double wRatio;
	double holeRatio;
public:
	double finalScore;
	double CenterX=160;

	double DetermineScore(double, double);
	double AreaOverBoundingArea();
	double GetDistance();

	Particle();
	virtual ~Particle();

	double GetAreaScreen(){return(areaScreen);}
	void SetAreaScreen(double setMe){areaScreen=setMe;}
	double GetAreaPixels(){return(areaPixels);}
	void SetAreaPixels(int setMe){areaPixels=setMe;}
	double GetAspect(){return(aspect);}
	void SetAspect(double setMe){aspect=setMe;}
	double GetBoundingRectTop(){return(BoundingRectTop);}
	void SetBoundingRectTop(double setMe){BoundingRectTop=setMe;}
	double GetBoundingRectLeft(){return(BoundingRectLeft);}
	void SetBoundingRectLeft(double setMe){BoundingRectLeft=setMe;}
	double GetBoundingRectRight(){return(BoundingRectRight);}
	void SetBoundingRectRight(double setMe){BoundingRectRight=setMe;}
	double GetBoundingRectBottom(){return(BoundingRectBottom);}
	void SetBoundingRectBottom(double setMe){BoundingRectBottom=setMe;}
	double GetMoment(){return(moment);}
	void SetMoment(double setMe){moment=setMe;}
	double GetRandomTest(){return(randomTest);}
	void SetRandomTest(double setMe){randomTest=setMe;}
	void SetWRatio(double setMe){wRatio=setMe;}
	double GetWRatio(){return(wRatio);}
	void SetHoleRatio(double setMe){holeRatio=setMe;}
	double GetHoleRatio(){return(holeRatio);}


};

#endif /* SRC_PARTICLE_H_ */
