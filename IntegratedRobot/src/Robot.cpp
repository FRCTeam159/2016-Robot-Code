#include "WPILib.h"
#include <AngleAdjuster.h>
#include <SRXConfigs/SRXSlave.h>
#include <SRXConfigs/SRXPosition.h>
#include <SRXConfigs/SRXSpeed.h>
#include <Holder/Holder.h>
#include <Launcher.h>
#include <Lidar.h>
#include <Loader.h>
#include <Particle.h>
#include <Shooter.h>
#include <TankDrive.h>
#include <Target.h>
#include <AngleAccelerometer.h>
#define TICKS_PER_CM 500
#define NO_TARGET 1234
class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

	Image *sendMe;
	USBCamera *forwardCamera, *reverseCamera;
	Target *horizontal;
	Particle *par;
	Joystick *stick;
	PWM *test;
	int visionState=0;
	CANTalon *leftDrive, *rightDrive;
	SRXSlave *leftSlave, *rightSlave;
	TankDrive *mydrive;
	CANTalon *shooterAngleMotor;
	AngleAccelerometer *shooterAngle;
	Lidar *lidar;

	SRXSpeed *flyWheelOne, *flyWheelTwo;
	Launcher *mylauncher;
	PIDController *vertAnglePID;

	bool pButton1=false, pButton2=false;
	void RobotInit()
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);
		forwardCamera= new USBCamera("cam2", true);
		reverseCamera= new USBCamera("cam3", true);

		forwardCamera->SetExposureManual(1);
		forwardCamera->OpenCamera();
		reverseCamera->OpenCamera();
		forwardCamera->StartCapture();
		reverseCamera->StartCapture();

		horizontal = new Target(forwardCamera);
		leftDrive= new CANTalon(0);
		rightDrive = new CANTalon(1);
		leftSlave = new SRXSlave(2,0);
		rightSlave = new SRXSlave(3,1);
		mydrive = new TankDrive(leftDrive, rightDrive, leftSlave, rightSlave, 1);
		flyWheelOne= new SRXSpeed(5,0,0,0,1);//zeros are PID, 1 is maxticks
		flyWheelTwo= new SRXSpeed(6,0,0,0,1);
		shooterAngleMotor = new CANTalon(7);
		shooterAngle = new AngleAccelerometer();
		vertAnglePID = new PIDController(.1, 0,0, shooterAngle, shooterAngleMotor);//INPUT CONSTANTS TODO
		mylauncher = new Launcher(flyWheelOne, flyWheelTwo, vertAnglePID);
		lidar = new Lidar(I2C::kMXP, 0x62);
		stick= new Joystick(0);

		sendMe=imaqCreateImage(IMAQ_IMAGE_HSL, 0);

		test =  new PWM(0);
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
		mydrive->ConfigAuto(0,0,0);
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
		mydrive->ConfigTeleop(0,0,0);
		visionState = 0;
	}

	void TeleopPeriodic()
	{
		visionState = visionStateMachine(visionState);
		bool button2=stick->GetRawButton(2);
		if(button2 && !pButton2)
		{
			if(visionState>3)
			{
				visionState=ExitLoop;
			}
			else if(visionState==GetForwardImage||visionState==SendForwardImage)
			{
				visionState=GetReverseImage;
			}
			else if(visionState==GetReverseImage||visionState==SendReverseImage)
			{
				visionState=GetForwardImage;
			}
		}
		pButton2=button2;

		bool button1=stick->GetRawButton(1);
		if(visionState<4)
		{
			if(button1&&!pButton1)
			{
				visionState = StartCalibrations;
			}
		}
		pButton1 = button1;
		mylauncher->Obey();

	}

	void TestPeriodic()
	{
		lw->Run();
	}
	enum TargetingStatus
	{
		GetForwardImage = 0,
		SendForwardImage=1,
		GetReverseImage = 2,
		SendReverseImage = 3,
		AcquireTargetImage = 4,
		ThresholdTargetImage = 5,
		CreateDebugImage = 6,
		ProcessTargetImage = 7,
		RequestConfirmation = 8,
		GetRangeFromLIDAR = 9,
		StartCalibrations = 10,
		WaitForCalibrations =11,
		ExitLoop = 12
	};
	int visionStateMachine(int state)
	{
		static int range;
		static float horizError;
		static bool firstCalibration;
		//states 0 and 1 get and send images from the forward camera
		if (state==GetForwardImage)
		{
			forwardCamera->GetImage(sendMe);
			state=SendForwardImage;
		}
		if(state==SendForwardImage)
		{
			CameraServer::GetInstance()->SetImage(sendMe);
			state=GetForwardImage;
		}
		//states 2 and 3 get and send images from reverse camera
		if (state==GetReverseImage)
		{
			reverseCamera->GetImage(sendMe);
			state=SendReverseImage;
		}
		if (state==SendReverseImage)
		{
			CameraServer::GetInstance()->SetImage(sendMe);
			state=GetReverseImage;
		}
		//states 4-8 acquire and process an image, then wait for dashboard confirmation
		if(state==StartCalibrations)
		{
			firstCalibration=true;
			mydrive->ConfigForPID();
			vertAnglePID->Enable();
		}
		if(state==GetRangeFromLIDAR)
		{
			range = lidar->GetDistance();
			float angle=shooterAngle->PIDGet();
			range=range*cos(angle*3.14/180);
			//calculate the required angle from the range here TODO
			//calculate the required flywheel speed here
			mylauncher->SetTargetSpeed(1);//in ticks/sec
			mylauncher->SetAngle(20);//target angle in degrees
			state=AcquireTargetImage;
		}
		if(state==AcquireTargetImage)
		{
			horizontal->AcquireImage();
			state=ThresholdTargetImage;
		}
		if(state==ThresholdTargetImage)
		{
			horizontal->ThresholdImage();
			state=CreateDebugImage;
		}
		if(state==CreateDebugImage)
		{

			horizontal->CreateDebugImage();
			state=ProcessTargetImage;
		}
		if(state==ProcessTargetImage)
		{
			int currentOffset=(horizontal->GetBestParticle())->CenterX-160;
			float targetOffset=horizontal->calculateTargetOffset(range);
			currentOffset=(currentOffset/320)*range;//convert pixels to centimeters
			targetOffset=(targetOffset/320)*range;
			horizError=atan(currentOffset/range)-atan(targetOffset/range);//get how many radians off we are
			horizError=(11*2.54)*TICKS_PER_CM;//convert angle to ticks (this will need a little tuning)
			mydrive->SetPosTargets(-1*horizError, horizError);//set drive targets
			state=WaitForCalibrations;
		}
		if(state==WaitForCalibrations)
		{
			mylauncher->AngleGood(2);//use this
			mylauncher->SpeedGood(200);//use this too
			if(true)//check to see if motors are close enough to target positions TODO
			{
				if(firstCalibration)
				{
					state=GetRangeFromLIDAR;
					firstCalibration=false;
				}

				else
				{
					state=RequestConfirmation;
				}

			}
			else
			{
				state=WaitForCalibrations;
			}

		}
		if(state==RequestConfirmation)// waits for operator confirmation
		{
			if(stick->GetRawButton(3))//operator rejects image
			{
				state=0;
			}
			if(stick->GetRawButton(4))//operator accepts imae
			{
				state=9;
			}
		}

		if(state==ExitLoop)
		{
			state=GetForwardImage;
			mydrive->ConfigTeleop(0,0,0);//TODO
			vertAnglePID->Disable();//TODO we need to decide how to zero the shooter when
		}//							  exiting the loop. Do we stall the exit until shooter is zeroed?
		return(state);
	}
};

START_ROBOT_CLASS(Robot)
