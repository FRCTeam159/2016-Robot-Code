#include "WPILib.h"
#include <SRXConfigs/SRXSlave.h>
#include <SRXConfigs/SRXPosition.h>
#include <SRXConfigs/SRXSpeed.h>
#include <Holder/Holder.h>
#include <Launcher.h>
#include <Sensors/Lidar.h>
#include <Loader/Loader.h>
#include <Particle.h>
#include <Shooter.h>
#include <TankDrive.h>
#include <Target.h>
#include <Sensors/AngleAccelerometer.h>
#include <Assignments.h>

#define TICKS_PER_CM 500
#define NO_TARGET 1234
#define HORIZONTAL_TARGETING 1//0 is by angle, 1 is by pixel difference and drivePID
#define AUT_DRIVE_P//TODO PID constants for the autonomous position control mode for the drive train
#define AUT_DRIVE_I
#define AUT_DRIVE_D

#define TEL_DRIVE_P 1//TODO PID constants for the teleop arcade drive for the drive train
#define TEL_DRIVE_I 0
#define TEL_DRIVE_D 0

#define CAM_DRIVE_P .05 //TODO PID constants for the external aiming pid for the drive train
#define CAM_DRIVE_I 0
#define CAM_DRIVE_D 0
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
	ShootAngleAccelerometer *shooterAngle;
	Holder *holder;
	Lidar *lidar;
	SRXSpeed *flyWheelOne, *flyWheelTwo;
	Launcher *mylauncher;
	PIDController *drivePID, *vertAnglePID;
	Loader *loader;

	int autoState;
	bool pButton1=false, pButton2=false, pButton3=false;
	void RobotInit()
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);

		forwardCamera= new USBCamera("cam2", true);
		reverseCamera= new USBCamera("cam1", true);

		forwardCamera->SetExposureManual(1);
		forwardCamera->OpenCamera();
		reverseCamera->OpenCamera();
		forwardCamera->StartCapture();
		reverseCamera->StartCapture();

		horizontal = new Target(forwardCamera);
		leftDrive= new CANTalon(CAN_LEFT_DRIVE);
		leftDrive->ConfigLimitMode(CANTalon::kLimitMode_SrxDisableSwitchInputs);
		rightDrive = new CANTalon(CAN_RIGHT_DRIVE);
		rightDrive->SetInverted(true);
		leftSlave = new SRXSlave(CAN_LEFT_SLAVE,CAN_LEFT_DRIVE);
		rightSlave = new SRXSlave(CAN_RIGHT_SLAVE,CAN_RIGHT_DRIVE);
		mydrive = new TankDrive(leftDrive, rightDrive, leftSlave, rightSlave, 1);
		drivePID= new PIDController(CAM_DRIVE_P,CAM_DRIVE_I ,CAM_DRIVE_D, horizontal, mydrive);
		drivePID->SetOutputRange(-1, 1);
		flyWheelOne= new SRXSpeed(CAN_FLYWHEEL_L,0,0,0,1);//zeros are PID, 1 is maxticks TODO
		flyWheelTwo= new SRXSpeed(CAN_FLYWHEEL_R,0,0,0,1);
		shooterAngleMotor = new CANTalon(CAN_SHOOT_ANGLE);
		shooterAngleMotor->ConfigRevLimitSwitchNormallyOpen(true);
		shooterAngleMotor->SetControlMode(CANTalon::kPercentVbus);
		shooterAngleMotor->SetInverted(false);
		shooterAngle = new ShootAngleAccelerometer(I2C::Port::kMXP);
//		vertAnglePID = new PIDController(.01, 0.0001,0, shooterAngle, shooterAngleMotor);//INPUT CONSTANTS TODO
//		vertAnglePID->SetOutputRange(-1,1);
//		vertAnglePID->SetToleranceBuffer(5);
		mylauncher = new Launcher(flyWheelOne, flyWheelTwo, shooterAngle, shooterAngleMotor);
		lidar = new Lidar(I2C::kMXP, 0x62);
		stick= new Joystick(0);
		holder = new Holder(HOLDER_GATE,HOLDER_PUSHER,IRSENSOR);
		loader = new Loader(CAN_LIFTER, CAN_ROLLER, I2C::Port::kMXP);
		sendMe=imaqCreateImage(IMAQ_IMAGE_HSL, 0);

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
		mydrive->ConfigAuto(0,0,0); //configuring the left and right motors for autonomous and takes an input of PID
		holder->AutonomousInit(); //configuring holder for autonomous, finds zero if it has not yet been found
		loader->SetLow(); //make sure the loader is in the low position
		autoState=1;
	}

	void AutonomousPeriodic()
	{
		if(autoSelected == autoNameCustom){
			//Custom Auto goes here
		} else {
			//Default Auto goes here

		}
		static int currentState=0;
		AutoStateMachine(currentState);
		mydrive->Obey(); //tank drive slaving, sets the targets of non slave wheels and sets slave wheel to targets of the master wheels
		loader->Obey(); //loader state machine, only has Waiting state.
		holder->AutoHold(); //holder state machine
	}

	void TeleopInit()
	{
//		mydrive->ConfigTeleop(TEL_DRIVE_P, TEL_DRIVE_I, TEL_DRIVE_D); //configuring for teleop with input for PID
		mydrive->ConfigForPID();
		visionState = 0;
		loader->SetLow();
		holder->TeleopInit(); //configuring for teleop
		flyWheelOne->SetControlMode(CANTalon::kPercentVbus);
		flyWheelTwo->SetControlMode(CANTalon::kPercentVbus);
//		vertAnglePID->SetSetpoint(20);
//		vertAnglePID->Disable();
	}

	void TeleopPeriodic()
	{
		visionState = visionStateMachine(visionState);
		bool button1=stick->GetRawButton(AIM);
		if(button1&&!pButton1)
		{
			if(visionState==GetForwardImage||visionState==SendForwardImage)
			{
//				visionState=StartCalibrations;
				holder->PushBall();
			}
			if(visionState==GetReverseImage||visionState==SendReverseImage)
			{
				loader->Continue();
			}
		}
		if(button1)
		{
			flyWheelOne->Set(.6);
			flyWheelTwo->Set(-.6);
		}
		else{
			flyWheelOne->Set(0);
			flyWheelTwo->Set(0);
		}
		pButton1 = button1;

		bool button2=stick->GetRawButton(SWITCH_CAMERA);
		if(button2&&!pButton2)
		{
			if(visionState>3)//in the auto aiming machine
			{
				visionState=ExitLoop;
			}
			else if (visionState==GetReverseImage||visionState==SendReverseImage)
			{
				if(!(loader->GetState()==Loader::LOW))//loader is doing something
					{loader->Cancel();}
				else
					{visionState=GetForwardImage;}//switch to forward camera
			}
			else if (visionState==GetForwardImage||visionState==SendForwardImage)
			{
				visionState=GetReverseImage;//switch to reverse camera
			}
		}
		pButton2=button2;

		//detect when we get the ball
		if((loader->GetState()==Loader::HIGH)&&holder->IsLoaded())
		{
			loader->SetLow();
			if(visionState==GetReverseImage||visionState==SendReverseImage)
			{
				visionState=GetForwardImage;
			}
		}
		//____BEGIN TEST CODE_______ TODO: Remove this
		static float targetAngle=20;
		static bool pbutton3 = false;
		static bool pbutton5 = false;

		bool button3=stick->GetRawButton(3);
		bool button5=stick->GetRawButton(5);
		if(button3&&!pbutton3)
		{
			targetAngle-=5;
			std::cout<<"target angle = "<<targetAngle<<std::endl;
		}
		if(button5&&!pbutton5)
		{
			targetAngle+=5;
			std::cout<<"target angle = "<<targetAngle<<std::endl;
		}
		pbutton3 = button3;
		pbutton5 = button5;
		shooterAngle->PIDGet();
		if(stick->GetRawButton(4))
		{
			mylauncher->SetAngle(targetAngle);
		}
//		if(button2)
//		{
//			std::cout<<"angle = "<<shooterAngle->PIDGet()<<std::endl;
//		}
		//___END TEST CODE_____
		if(visionState==GetForwardImage||visionState==SendForwardImage)//using forward camera
		{
			mydrive->ArcadeDrive(stick);
		}
		else if (visionState==GetReverseImage||visionState==SendReverseImage)
		{//if using reverse camera, drive in reverse
			mydrive->RevArcadeDrive(stick);
		}
		/*if(!vertAnglePID->IsEnabled()) //pid controller is not enabled
		{//TODO make sure the correct limit switches are plugged into this motor
			shooterAngleMotor->Set(-.2);//it should stop automatically when it hits the limit switch
		}*/

		loader->Obey(); //loader state machine
		holder->AutoHold(); //holder state machine
		mylauncher->Obey(); //setting motors
		mydrive->Obey(); //more motor slave stuff
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
		ExitLoop = 12,
		ShootBall = 13,
		CheckBall = 14,
		SetInitialAngle = 15,
	};
	int visionStateMachine(int state)
	{
		static int range;
		static float horizError; //used in horizontal targeting
		static bool firstCalibration;
		static Particle *best=NULL;
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
			std::cout<<"state: StartCalibrations"<<std::endl;
			range = lidar->GetDistance(); //getting distance
			if(!range==1234)
			{
				firstCalibration=true;
//				vertAnglePID->Enable(); //enabling PID controller
//				vertAnglePID->SetSetpoint(20); //setting the PID controller target
				mylauncher->SetAngle(20);
#if HORIZONTAL_TARGETING ==1
				mydrive->ConfigForPID();
#endif
#if HORIZONTAL_TARGETING ==0
				mydrive->ConfigAuto(0,0,0);
#endif
				state= SetInitialAngle;
			}
			else
			{
				state=StartCalibrations;
			}
		}
		if(state==SetInitialAngle)
		{
			std::cout<<"state: SetInitialAngle"<<std::endl;
			if(mylauncher->AngleGood(1))
				state=GetRangeFromLIDAR;
		}
		if(state==GetRangeFromLIDAR)
		{
			std::cout<<"state: GetRangeFromLIDAR"<<std::endl;
			int confirmrange = lidar->GetDistance();
			float angle=shooterAngle->PIDGet();
			confirmrange=confirmrange*cos(angle*3.14/180);
			if(fabs(confirmrange-range)<5){
				range=(range+confirmrange)/2;
				mylauncher->Aim(range/100);
				state=AcquireTargetImage;
			}
			else if(confirmrange>range)
			{
				range=confirmrange;
				mylauncher->Aim(range/100);
				state=AcquireTargetImage;
			}
			else
			{
				state=GetRangeFromLIDAR;
			}
		}
		if(state==AcquireTargetImage)
		{
			std::cout<<"state: AcquireTargetImage"<<std::endl;
			horizontal->AcquireImage();
			state=ThresholdTargetImage;
		}
		if(state==ThresholdTargetImage)
		{
			std::cout<<"state: ThresholdTargetImage"<<std::endl;
			horizontal->ThresholdImage();
			state=ProcessTargetImage;
		}

		if(state==ProcessTargetImage)
		{
			std::cout<<"state: ProcessTargetImage"<<std::endl;
			float targetOffset=horizontal->calculateTargetOffset(range);
			best=horizontal->GetBestParticle();
#if HORIZONTAL_TARGETING == 0
			int currentOffset=(horizontal->GetBestParticle())->CenterX-160;
			currentOffset=(currentOffset/320)*range;//convert pixels to centimeters
			targetOffset=(targetOffset/320)*range;
			horizError=atan(currentOffset/range)-atan(targetOffset/range);//get how many radians off we are
			horizError=(11*2.54)*TICKS_PER_CM;//convert angle to ticks (this will need a little tuning)
			mydrive->SetPosTargets(-1*horizError, horizError);//set drive targets
#endif
#if HORIZONTAL_TARGETING == 1
			if(best->CenterX!=1234)//1234 is error
			{
				if(!drivePID->IsEnabled()) //if drive PID controller is not enabled
				{
					drivePID->Enable(); //enable drive PID controller
				}
				drivePID->SetSetpoint(targetOffset); //set PID controller target
			}
			else
			{
				drivePID->Disable();
			}
#endif
			state=WaitForCalibrations;
		}
		if(state==WaitForCalibrations)
		{
			std::cout<<"state: WaitForCalibrations"<<std::endl;
			bool good = mylauncher->AngleGood(2);//use this
			good = good && mylauncher->SpeedGood(200);//use this too
			good = good && drivePID->GetError()<3 && drivePID->IsEnabled();//this is also handy
			if(good)//check to see if motors are close enough to target positions
			{
				if(firstCalibration)
				{
					state=AcquireTargetImage;
					firstCalibration=false;
				}

				else
				{
					state=CreateDebugImage;
				}

			}
			else
			{
				state=AcquireTargetImage;

			}

		}
		if(state==CreateDebugImage)
		{
			std::cout<<"state: CreateDebugImage"<<std::endl;
			horizontal->CreateDebugImage();
			horizontal->AnnotateDebugImage(best);
			horizontal->SendDebugImage();
			state=RequestConfirmation;
		}
		if(state==RequestConfirmation)// waits for operator confirmation
		{//the operator can exit loop by hitting button 2 (controlled in Teleop periodic)
			std::cout<<"state: RequestConfirmation"<<std::endl;
			if(stick->GetRawButton(AIM))//operator accepts image
			{
				state=ShootBall;
			}
		}
		if(state==ShootBall)
		{
			std::cout<<"state: ShootBall"<<std::endl;
			holder->PushBall(); //pushes ball with pusher motor
			state=CheckBall;
		}
		if(state==CheckBall)
		{
			std::cout<<"state: CheckBall"<<std::endl;
			if(holder->CheckPushed())
			{
				state=ExitLoop;
			}
			else
			{
				state=CheckBall;
			}
		}
		if(state==ExitLoop)
		{
			std::cout<<"state: ExitLoop"<<std::endl;
			drivePID->Disable();
			mylauncher->SetTargetSpeed(0);
			mylauncher->SetAngle(0);
//			mydrive->ConfigTeleop(TEL_DRIVE_P,TEL_DRIVE_I,TEL_DRIVE_D); //configuring back to init config
			if(mylauncher->AngleGood(1))
			{
				state=GetForwardImage;
			}
			else
			{
				state = ExitLoop;
			}
//			vertAnglePID->Disable();
		}
		return(state);
	}
	int AutoStateMachine(int state)
	{
		static bool firstCalibration;
		static float range;
		static Particle *best;
		if(state==StartCalibrations)
		{
			firstCalibration=true;
#if HORIZONTAL_TARGETING ==1
			mydrive->ConfigForPID();
#endif
#if HORIZONTAL_TARGETING ==0
			mydrive->ConfigAuto(0,0,0);
#endif
			range = lidar->GetDistance();
			vertAnglePID->Enable();
			vertAnglePID->SetSetpoint(20); //set PID controller target
			state= SetInitialAngle;
		}
		if(state==SetInitialAngle)
		{
			if(vertAnglePID->GetAvgError()<1)
				state=GetRangeFromLIDAR;
		}
		if(state==GetRangeFromLIDAR)
		{
			int confirmrange = lidar->GetDistance();
			float angle=shooterAngle->PIDGet();
			confirmrange=confirmrange*cos(angle*3.14/180);
			range=(range+confirmrange)/2;
			mylauncher->Aim(range/100);
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
			float targetOffset=horizontal->calculateTargetOffset(range);
			best=horizontal->GetBestParticle();
#if HORIZONTAL_TARGETING == 0
			int currentOffset=(horizontal->GetBestParticle())->CenterX-160;
			currentOffset=(currentOffset/320)*range;//convert pixels to centimeters
			targetOffset=(targetOffset/320)*range;
			horizError=atan(currentOffset/range)-atan(targetOffset/range);//get how many radians off we are
			horizError=(11*2.54)*TICKS_PER_CM;//convert angle to ticks (this will need a little tuning)
			mydrive->SetPosTargets(-1*horizError, horizError);//set drive targets
#endif
#if HORIZONTAL_TARGETING == 1
			if(best->CenterX!=1234)//1234 is error
			{
				if(!drivePID->IsEnabled()) //if PID controller is not enabled
				{
					drivePID->Enable(); //enable PID controller
				}
				drivePID->SetSetpoint(targetOffset); //set PID controller target to targetOffset
			}
			else
			{
				drivePID->Disable(); //disable PID controller
			}
#endif
			state=WaitForCalibrations;
		}
		if(state==WaitForCalibrations)
		{
			bool good = mylauncher->AngleGood(2);//use this
			good = good && mylauncher->SpeedGood(200);//use this too
			good = good && drivePID->GetError()<3 && drivePID->IsEnabled();//this is also handy
			if(good)//check to see if motors are close enough to target positions
			{
				if(firstCalibration)
				{
					state=AcquireTargetImage;
					firstCalibration=false;
				}
				else
				{
					state=ShootBall;
				}
			}
			else
			{
				state=AcquireTargetImage;
			}
		}
		if(state==ShootBall)
		{
			holder->PushBall(); //push ball into shooter with roller motors
			state=CheckBall;
		}
		if(state==CheckBall)
		{
			if(holder->CheckPushed())
			{
				state=ExitLoop;
			}
			else
			{
				state=CheckBall;
			}
		}
		if(state==ExitLoop)
		{
			state=GetForwardImage;
			drivePID->Disable();
			mylauncher->SetTargetSpeed(0);
			mydrive->ConfigTeleop(0,0,0);//TODO
			vertAnglePID->Disable();
		}
		return state;
	}

};

START_ROBOT_CLASS(Robot)
//this is potentially something to sue for moving before starting aiming
/*if(autoState==1)
{
	bool done= mydrive->CloseEnough(200);//TODO change this
	//done = done && loader->close enough TODO
	if (done)
	{
		mydrive->ZeroMotors();
		autoState=2;
	}

}
if (autoState==2)
{
	mydrive->SetPosTargets(2400,2400);//TODO
	if(mydrive->CloseEnough(200))
	{
		autoState=3;
		mydrive->ZeroMotors();
	}
}
if (autoState==3)
{
	mydrive->SetPosTargets(500,-500);
	if(mydrive->CloseEnough(30))//TODO
	{
		mydrive->SetPosTargets(0,0);
		mydrive->ConfigForPID();
		autoState=GetRangeFromLIDAR;
	}
}


 *
 */
