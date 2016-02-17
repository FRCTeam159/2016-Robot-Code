#include "WPILib.h"
#include "Loader/Loader.h"
#include "Assignments.h"
#define LOW_ANGLE 0
#define MID_ANGLE 30
#define HIGH_ANGLE 50

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	Loader *loader;
	Joystick *joystick;

	void RobotInit()
	{
		chooser = new SendableChooser();
		SmartDashboard::PutData("Auto Modes", chooser);
		loader = new Loader(1,2,I2C::kOnboard);
		joystick = new Joystick(0);
	}


	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{

	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
