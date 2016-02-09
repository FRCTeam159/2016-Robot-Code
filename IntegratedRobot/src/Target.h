/*
 * Target.h
 *
 *  Created on: Jan 29, 2016
 *      Author: alpin
 */

#ifndef SRC_TARGET_H_
#define SRC_TARGET_H_
#include<Particle.h>
#include <WPILib.h>
struct AreaHolder
{
	int id;
	double area;
};

#define MAX_PARTICLES 10
class Target {
private:
	USBCamera *camera;
	Image *frame, *filteredFrame, *debugFrame;

	AreaHolder sortMe[MAX_PARTICLES];
	Particle allParticles[MAX_PARTICLES];

	Range Hue_Range= {0, 255};
	Range Sat_Range={0, 255};
	Range Val_Range={180, 255};
	ParticleFilterCriteria2 criteria={IMAQ_MT_AREA_BY_IMAGE_AREA, 0, .3, false, true};
	ParticleFilterOptions2 filterOptions = {0,0,0,1};

	int particleCount;

	void PostThresholdProcessing();
	void InitAndSortArrays();
	void ScoreParticles();
public:
	Target(USBCamera*);
	void AcquireImage();
	void ThresholdImage();
	Particle GetBestParticle();
	void CreateDebugImage();
	void AnnotateDebugImage(Particle);
	void SendDebugImage();
	float calculateTargetOffset(float);
	virtual ~Target();
};


#endif /* SRC_TARGET_H_ */
