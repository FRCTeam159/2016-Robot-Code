#include "WPILib.h"
#include "SRXSpeed.h"
#include "AngleAccelerometer.h"
#include "Launcher.h"
#include "Lidar.h"
#define P_CONSTANT .03
class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
	SRXSpeed *leftWheel, *rightWheel;
	CANTalon *angleMotor;
	AngleAccelerometer *angle;
	Launcher *launch;
	Joystick *stick;
	PIDController *anglePID;
	Lidar *lidar;
	void RobotInit()
	{	//pusher = 2, gate = 4
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);
		lidar = new Lidar(I2C::kMXP, 0x62);
		leftWheel = new SRXSpeed(3, P_CONSTANT, 0.0, 0.0, 1);
		rightWheel = new SRXSpeed(1, P_CONSTANT, 0.0, 0.0, 1);
		leftWheel->SetInverted(true);
		stick = new Joystick(0);
		angle = new AngleAccelerometer(I2C::kOnboard);
		angleMotor = new CANTalon(5);
		anglePID = new PIDController(.05, 0, 0, angle, angleMotor);
		launch = new Launcher(leftWheel, rightWheel, anglePID);
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

	}

	void TeleopPeriodic()
	{
		static int targetSpeed = 0;
		targetSpeed+= stick->GetY()*-3;
		static int fakerange = 0;
		if (stick->GetRawButton(2))
		{
			targetSpeed=0;
			fakerange = 250;
		}
		if (stick->GetRawButton(3))
		{
			targetSpeed += 150;
		}
		if (stick->GetRawButton(4))
		{
			targetSpeed -= 150;
		}
		if(stick->GetRawButton(5))
		{
			fakerange+=3;
		}
		if(stick->GetRawButton(6))
		{
			fakerange-=3;
		}
		launch->Obey();
		launch->SetTargetSpeed(targetSpeed);
		float currAngle= angle->PIDGet();
		float range = fakerange*cos(currAngle*3.1415/180);
		std::cout<<"range estimate = "<<range<<std::endl;
		launch->Aim(range/100.0);
		int current =rightWheel->GetEncVel();
		std::cout<<targetSpeed<<"  "<<current<<std::endl;
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
