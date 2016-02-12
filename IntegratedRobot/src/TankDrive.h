/*
 * TankDrive.h
 *
 *  Created on: Jan 29, 2016
 *      Author: alpin
 */

#ifndef SRC_TANKDRIVE_H_
#define SRC_TANKDRIVE_H_
#include "WPILib.h"
class SRXSlave;
class TankDrive: public PIDOutput {
private:
	int config=0;
	CANTalon *leftMotor, *rightMotor;
	SRXSlave *leftSlave, *rightSlave;
	float deadband(float, float);
	float leftTarget=0, rightTarget=0;
	int maxTicks;
public:
	TankDrive(CANTalon*, CANTalon*, SRXSlave*, SRXSlave* ,int);
	void SetPosTargets(float, float);
	void ArcadeDrive(Joystick*);
	void ConfigAuto(float, float, float);
	void ConfigTeleop(float, float, float);
	bool CloseEnough(float);
	void Obey();
	void PIDWrite(float output);
	void ConfigForPID();
	virtual ~TankDrive();
	float previousPID=0;
};

#endif /* SRC_TANKDRIVE_H_ */
