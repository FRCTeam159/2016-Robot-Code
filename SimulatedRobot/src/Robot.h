/*
 * Robot.h
 *
 *  Created on: Jun 3, 2014
 *      Author: alex
 */

#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

#include "WPILib.h"
#include "Commands/Command.h"

#include "Subsystems/DriveTrain.h"
#include "Subsystems/Holder.h"
#include "Subsystems/Shooter.h"
#include "OI.h"

class Robot: public IterativeRobot {
public:
	static DriveTrain* drivetrain;
	static BallHolder* holder;
	static Shooter* shooter;
	static OI* oi;

private:
	LiveWindow *lw;
	~Robot();
	void RobotInit();
	void AutonomousInit();
	void AutonomousPeriodic();
	void TeleopInit();
	void TeleopPeriodic();
	void TestPeriodic();
};


#endif /* ROBOT_H_ */
