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


	void RobotInit()
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);
		holder=new Holder(HOLDER_GATE,HOLDER_PUSHER,REVGATELIMIT,FWDGATELIMIT,IRSENSOR);
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
		autoSelected = *((std::string*)chooser->GetSelected());
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if(autoSelected == autoNameCustom){
			//Custom Auto goes here
		} else {
			//Default Auto goes here
		}
	}

	void AutonomousPeriodic()
	{
		if(autoSelected == autoNameCustom){
			//Custom Auto goes here
		} else {
			//Default Auto goes here
		}
	}

	void TeleopInit()
	{
		holder->TeleopInit();
	}

	void TeleopPeriodic()
	{
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
