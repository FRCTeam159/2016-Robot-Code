#include "WPILib.h"
#include "Loader/Loader.h"
#include "Holder/Holder.h"
#include "Holder/Holder.cpp"
#include "Assignments.h"
#include <Sensors/AngleAccelerometer.h>
#define LOW_ANGLE 0
#define MID_ANGLE 30
#define HIGH_ANGLE 50

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	Loader *loader;
	Holder *holder;
	Joystick *joystick;
	int state;
	bool pButton;
	bool pButton2;

	void RobotInit()
	{
		chooser = new SendableChooser();
		SmartDashboard::PutData("Auto Modes", chooser);
		loader = new Loader(3,4,I2C::kOnboard);
		joystick = new Joystick(0);
		holder = new Holder(5,8,1,2,0);
		state = ARMUP;
	}

	enum {
		ARMDOWN,
		ARMUP
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
	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
		loader->TeleopInit();
	}

	void TeleopPeriodic()
	{
		/*bool button3 = joystick->GetRawButton(3);
		bool button4 = joystick->GetRawButton(4);
		bool button2 = joystick->GetRawButton(2);
		if(button3){
			loader->SetLowPosition();
		}
		if(button4){
			loader->GrabBall();
		}
		if(button2){
			loader->Waiting();
		}*/
		bool button = joystick->GetRawButton(2);
		if((button != pButton) && (button == 1)){
			printf("buttonstate:%d\n", button);
			switch(state){
			case ARMDOWN:
				loader->SetLowPosition();
				printf("ARMDOWN\n");
				state=ARMUP;
				break;
			case ARMUP:
				loader->GrabBall();
				printf("ARMUP\n");
				state=ARMDOWN;
				/*			if(){
					loader->SetLowPosition();
			}*/
				break;
			}
		}
		loader->Obey();
		pButton = button;
	}


	void TestPeriodic()
	{
		lw->Run();
	}

	void DisabledInit()
	{
		loader->Waiting();
	}

};

START_ROBOT_CLASS(Robot)
