#include "WPILib.h"
#include "Holder/Holder.h"
#include "Assignments.h"
//#define GATEMOTOR 1
//#define PUSHERMOTOR 2
//#define REVGATELIMIT 1
//#define FWDGATELIMIT 2
//#define IRSENSOR 0

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
	Holder *holder;
	Joystick *joystick;
	int state;
	bool pButton;


	void RobotInit()
	{
		joystick = new Joystick(0);
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);
		holder=new Holder(HOLDER_GATE,HOLDER_PUSHER,IRSENSOR);
		state = WAIT_FOR_BUTTON;
	}

	enum {
		SHOOT_BALL,
		CHECK_BALL,
		WAIT_FOR_BUTTON
	};

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */

	//immediately fire for testing
	void AutonomousInit()
	{
		holder->AutonomousInit();
	}

	void AutonomousPeriodic()
	{
		holder->AutoHold();
	}

	void TeleopInit()
	{
		holder->TeleopInit();
		state = WAIT_FOR_BUTTON;
		pButton = false;
	}

	void TeleopPeriodic()
	{
		bool button = joystick->GetRawButton(1);

		if(button && !pButton){
			holder->PushBall();
		}
		pButton=button;
		holder->TeleopPeriodic();
	}

	void TestInit()
	{
		//lw->SetEnabled(false);
		//holder->TestInit();
		//Scheduler::GetInstance()->SetEnabled(true);
//		printf("TestInit\n");
//		Wait(0.2);
	}

	void TestPeriodic()
	{
		//bool isEnabled = false;
		lw->Run();
		//holder->TestPeriodic();
		//printf("TestPeriodic\n");

	}

	void DisabledPeriodic()
	{
		//holder->Disable();
	}
};

START_ROBOT_CLASS(Robot)
