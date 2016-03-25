#include "WPILib.h"
#define MASTER_ID 11
#define SLAVE_ID 3
//#define USE_SLAVE
class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
	//Victor *myMotor;
	CANTalon *myMotor;
	CANTalon *slave;
	Joystick *stick;
	void RobotInit()
	{
//		chooser = new SendableChooser();
//		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
//		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
//		SmartDashboard::PutData("Auto Modes", chooser);
		myMotor = new CANTalon(MASTER_ID);
		myMotor->ConfigLimitMode(CANTalon::LimitMode::kLimitMode_SrxDisableSwitchInputs);
		myMotor->ConfigRevLimitSwitchNormallyOpen(true);
		myMotor->SetInverted(false);
#ifdef USE_SLAVE
		slave = new CANTalon(SLAVE_ID);
		slave->SetControlMode(CANTalon::kFollower);
		slave->EnableControl();
#endif
		stick = new Joystick(0);
		myMotor->ClearStickyFaults();
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
		myMotor->SetControlMode(CANTalon::kPercentVbus);
		myMotor->ClearStickyFaults();
	}

	void TeleopPeriodic()
	{
		float target = stick->GetY()*-1;
		myMotor->Set(target);
		std::cout<<"target: "<<target<<std::endl;
#ifdef USE_SLAVE
		slave->Set(MASTER_ID);
#endif
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
