#include "WPILib.h"
#include "Lidar.h"
#include "Target.h"
#include "Particle.h"
#include "AngleAccelerometer.h"
#include "TankDrive.h"
#include "SRXSlave.h"
#define NO_TARGET 1234
class Robot: public IterativeRobot
{
private:
	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
	Lidar *laser;
	Target *myTarget;
	USBCamera *forward;
	Relay *light;
	AngleAccelerometer *angle;
	CANTalon *shooterAngleMotor, *leftMotor, *rightMotor;
	TankDrive *mydrive;
	SRXSlave *placeholder=NULL, *placeholdert=NULL;
	PIDController *mypid, *drivepid;
	Joystick *stick;
	float pidtarget=20;
	void RobotInit()
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);
		laser=new Lidar(I2C::kMXP, 0x62);
		forward= new USBCamera("cam2", true);
		forward->SetExposureManual(1);
		forward->OpenCamera();
		forward->StartCapture();
		myTarget= new Target(forward);
		light= new Relay(0);
		angle = new AngleAccelerometer();
		shooterAngleMotor= new CANTalon(0);
		shooterAngleMotor->SetControlMode(CANTalon::kPercentVbus);
		mypid= new PIDController(1,0,0, angle, shooterAngleMotor);
		stick = new Joystick(0);
		leftMotor= new CANTalon(42);
		rightMotor = new CANTalon(43);
		leftMotor->SetControlMode(CANTalon::kPercentVbus);
		rightMotor->SetControlMode(CANTalon::kPercentVbus);
		mydrive = new TankDrive(leftMotor, rightMotor, placeholder, placeholdert,1);
		drivepid = new PIDController(.01,0,0,myTarget, mydrive);
		std::cout<<"finished init"<<std::endl;
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
		mypid->Enable();
		drivepid->Enable();
		drivepid->SetSetpoint(0);
		std::cout<<"finsihed teleop init"<<std::endl;
	}

	void TeleopPeriodic()
	{
		light->Set(Relay::kForward);
		static int state=0;
		static float range;
		static float hRange;
		pidtarget+= stick->GetY()*.5;
		mypid->SetSetpoint(pidtarget);
		std::cout<<"last PID output = "<<mydrive->previousPID<<std::endl;
		if(state==0)
		{
			range = laser->GetDistance();
			hRange =range* cos(angle->PIDGet()*3.1415/180);
			std::cout<<"range = "<<range<<std::endl;
			state=1;
		}
		if(state==1)
		{
			myTarget->AcquireImage();
			state=2;
		}
		if(state==2)
		{
			myTarget->ThresholdImage();
			state=3;
		}
		if(state==3)
		{
			myTarget->CreateDebugImage();
			state=4;
		}
		if(state==4)
		{
			Particle *best=myTarget->GetBestParticle();
			float currentOffset=best->CenterX-160;
			if(currentOffset==NO_TARGET-160)
			{
				drivepid->SetSetpoint(currentOffset);
			}
			else{
				drivepid->SetSetpoint(0);
			}
			std::cout<<"error in pixels = "<<currentOffset<<std::endl;
			float targetOffset=myTarget->calculateTargetOffset(hRange);
			currentOffset=(currentOffset/320)*2*hRange*.39;//convert pixels to centimeters
			targetOffset=(targetOffset/320)*2*hRange*.39;
			float COdeg=atan(currentOffset/hRange)*180/3.14;
			float TOdeg= atan(targetOffset/hRange)*180/3.14;
			std::cout<<"current offset = "<<COdeg<<"target offset = "<<TOdeg<<std::endl;
			double horizError=atan(currentOffset/hRange)-atan(targetOffset/hRange);//get how many radians off we are
			std::cout<<"error (radians) = "<<horizError<<std::endl;
			state=5;
		}
		if(state==5)
		{
			myTarget->AnnotateDebugImage(myTarget->GetBestParticle());
			myTarget->SendDebugImage();
			state=0;
		}

//		horizError=(11*2.54)*TICKS_PER_CM;//convert angle to ticks (this will need a little tuning)
//		mydrive->SetPosTargets(-1*horizError, horizError);//set drive targets
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
