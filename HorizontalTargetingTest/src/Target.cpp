/*
 * Target.cpp
 *
 *  Created on: Jan 29, 2016
 *      Author: alpin
 */

#include <Target.h>
#include "string.h"
#include <WPILib.h>
#include <Particle.h>
#define CAMERA_OFFSET 10
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
Target::Target(USBCamera* a){
	frame = imaqCreateImage(IMAQ_IMAGE_HSL, 0);
	filteredFrame = imaqCreateImage(IMAQ_IMAGE_U8, 0);
	debugFrame= imaqCreateImage(IMAQ_IMAGE_U8, 0);
	camera=a;
	// TODO Auto-generated constructor stub

}

Target::~Target() {
	// TODO Auto-generated destructor stub
	m_pidSource=PIDSourceType::kDisplacement;
}

void Target::AcquireImage()
{
	camera->GetImage(frame);
}

void Target::ThresholdImage()
{
	imaqColorThreshold(filteredFrame, frame, 255, IMAQ_HSV, &Hue_Range, &Sat_Range, &Val_Range);
}

void Target::PostThresholdProcessing()
{
	imaqParticleFilter4(filteredFrame, filteredFrame, &criteria, 1, &filterOptions, NULL, NULL); //LOOK AT THIS IF YOU HAVE SPARE TIME
	imaqCountParticles(filteredFrame, 1, &particleCount);
	particleCount=fmin(particleCount, MAX_PARTICLES);//avoid errors
}

void Target::InitAndSortArrays()
{
	for(int index=0;index<particleCount;index++)//measure areas and initialize the sortMe array
	{
		double pain;
		imaqMeasureParticle(filteredFrame, index, 0, IMAQ_MT_AREA_BY_IMAGE_AREA, &pain);
		allParticles[index].SetAreaScreen(pain);
		sortMe[index].area=pain;
		sortMe[index].id=index;
	}
	//sort by area
	for(int index=0;index<particleCount;index++){//just sort the thing manually
		for(int place=0;place<particleCount-1-index;place++){
			if(sortMe[place].area<sortMe[place+1].area){
				AreaHolder a=sortMe[place];
				sortMe[place]=sortMe[place+1];
				sortMe[place+1]=a;
			}
		}
	}
}

void Target::ScoreParticles()
{
	for(int index=0;index<particleCount;index++)
	{
		if(allParticles[sortMe[index].id].GetAreaScreen()!=0)
		{
			//get values for scoring
			double pain;
			imaqMeasureParticle(filteredFrame, sortMe[index].id,0,IMAQ_MT_WADDEL_DISK_DIAMETER, &(pain));
			double waddel=pain;
			imaqMeasureParticle(filteredFrame, sortMe[index].id,0,IMAQ_MT_MAX_FERET_DIAMETER, &(pain));
			allParticles[sortMe[index].id].SetWRatio(pain/waddel);
			imaqMeasureParticle(filteredFrame, sortMe[index].id, 0, IMAQ_MT_AREA, &(pain));
			allParticles[sortMe[index].id].SetAreaPixels(pain);
			imaqMeasureParticle(filteredFrame, sortMe[index].id, 0, IMAQ_MT_HOLES_AREA, &(pain));
			allParticles[sortMe[index].id].SetHoleRatio((allParticles[sortMe[index].id].GetAreaPixels()-(2*pain))/allParticles[sortMe[index].id].GetAreaPixels());
			imaqMeasureParticle(filteredFrame,sortMe[index].id,0,IMAQ_MT_RATIO_OF_EQUIVALENT_RECT_SIDES,&(pain));
			allParticles[sortMe[index].id].SetAspect(pain);
			//get bounding rectangle
			imaqMeasureParticle(filteredFrame, sortMe[index].id, 0, IMAQ_MT_BOUNDING_RECT_TOP, &(pain));
			allParticles[sortMe[index].id].SetBoundingRectTop(pain);
			imaqMeasureParticle(filteredFrame, sortMe[index].id, 0, IMAQ_MT_BOUNDING_RECT_LEFT, &(pain));
			allParticles[sortMe[index].id].SetBoundingRectLeft(pain);
			imaqMeasureParticle(filteredFrame, sortMe[index].id, 0, IMAQ_MT_BOUNDING_RECT_BOTTOM, &(pain));
			allParticles[sortMe[index].id].SetBoundingRectBottom(pain);
			imaqMeasureParticle(filteredFrame, sortMe[index].id, 0, IMAQ_MT_BOUNDING_RECT_RIGHT, &(pain));
			allParticles[sortMe[index].id].SetBoundingRectRight(pain);
			//score the particle
			double score1=allParticles[sortMe[index].id].DetermineScore(.3, allParticles[sortMe[index].id].AreaOverBoundingArea());
			double score2=allParticles[sortMe[index].id].DetermineScore(2.23, allParticles[sortMe[index].id].GetWRatio());
			double score3=allParticles[sortMe[index].id].DetermineScore(35, allParticles[sortMe[index].id].GetAspect());
			double score4=allParticles[sortMe[index].id].DetermineScore(1, allParticles[sortMe[index].id].GetHoleRatio());
			allParticles[sortMe[index].id].finalScore=(score1+score2+score3+score4)/4;
		}
	}
}

Particle* Target::GetBestParticle()
{
	PostThresholdProcessing();
	InitAndSortArrays();
	ScoreParticles();

	bestScoreId=sortMe[0].id;
	double holder;
	if(particleCount>0){
		for(int index=1; index<particleCount+1;index++){
			if(allParticles[bestScoreId].finalScore<allParticles[sortMe[index].id].finalScore)
				bestScoreId=sortMe[index].id;
		}
		//get best particle's x position
		double pain;
		imaqMeasureParticle(filteredFrame, bestScoreId,0, IMAQ_MT_CENTER_OF_MASS_X, &(pain));
		allParticles[bestScoreId].CenterX=pain;
		return(&allParticles[bestScoreId]);
	}
	else{
		std::cout<<"No Particles!";
		Particle err;
		err.finalScore=0;
		return(&err);
	}

}

void Target::CreateDebugImage()
{
	imaqDuplicate(debugFrame, frame);
}

void Target::AnnotateDebugImage(Particle* best)
{
	Rect box;
	box.top=best->GetBoundingRectTop();
	box.left=best->GetBoundingRectLeft();
	box.height=abs(best->GetBoundingRectBottom()-box.top);
	box.width=abs(best->GetBoundingRectRight()-box.left);
	imaqDrawShapeOnImage(debugFrame, debugFrame, box, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_RECT, 5000.8f);
}

void Target::SendDebugImage()
{
	CameraServer::GetInstance()->SetImage(debugFrame);
}

float Target::calculateTargetOffset(float range)
{
	float offsetPixels=(CAMERA_OFFSET/(2*range*.39))*SCREEN_WIDTH;// .39 is a placeholder for the sine of half the horizontal view angle
	return(offsetPixels);
}

double Target::PIDGet()
{
	return(allParticles[bestScoreId].CenterX-160);
}
void Target::SetPIDSourceType(PIDSourceType type)
{
	m_pidSource=type;
}
