/*
 * OI.cpp
 *
 *  Created on: Jun 3, 2014
 *      Author: alex
 */

#include "OI.h"
#include "Commands/ToggleMode.h"
#include "Commands/ShootBall.h"
#include "Commands/StepShooterAngle.h"
#include "Commands/StepLoaderAngle.h"
#include "Commands/LoadBall.h"
#include "Commands/ToggleGate.h"

int OI::mode=SHOOTING;

OI::OI() :
		rightBtnCmnd1(this),rightBtnCmnd2(this),leftBtnCmnd1(this),leftBtnCmnd2(this),upBtnCmnd1(this),upBtnCmnd2(this)
{

	joy= new Joystick(0);

    // Create some buttons
    d_right = new JoystickButton(joy, 2);
    d_left= new JoystickButton(joy, 3);
    d_down= new JoystickButton(joy, 1);
    d_up = new JoystickButton(joy, 4);

    // bind down button to mode toggle

    d_down->ToggleWhenPressed(new ToggleMode());

    // bind commands based on current mode
    // case 1: SHOOTER mode
    rightBtnCmnd1.WhenPressed(new StepShooterAngle(10));
    leftBtnCmnd1.WhenPressed(new StepShooterAngle(-10));
    upBtnCmnd1.WhenPressed(new ShootBall());

    // case 1: LOADER mode
    rightBtnCmnd2.WhenPressed(new StepLoaderAngle(10));
    leftBtnCmnd2.WhenPressed(new StepLoaderAngle(-10));
    //upBtnCmnd2.WhenPressed(new LoadBall());
    upBtnCmnd2.WhenPressed(new ToggleGate());

}

Joystick* OI::GetJoystick() {
	return joy;
}
