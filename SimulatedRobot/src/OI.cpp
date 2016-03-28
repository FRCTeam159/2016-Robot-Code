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
#include "Commands/ToggleRollers.h"

int OI::mode=SHOOTING;

OI::OI() :
		rightBtnCmnd1(this),
		rightBtnCmnd2(this),
		leftBtnCmnd1(this),
		leftBtnCmnd2(this),
		upBtnCmnd1(this),
		upBtnCmnd2(this),
		downBtnCmnd1(this),
		downBtnCmnd2(this)
{

	joy= new Joystick(0);

    // Create some buttons
    d_right = new JoystickButton(joy, 2);
    d_left= new JoystickButton(joy, 3);
    d_down= new JoystickButton(joy, 1);
    d_up = new JoystickButton(joy, 4);

    d_mode = new JoystickButton(joy, 6);

    // bind down button to mode toggle

    d_mode->ToggleWhenPressed(new ToggleMode());

    // bind commands based on current mode
    // case 1: SHOOTER mode
    rightBtnCmnd1.WhenPressed(new StepShooterAngle(10));
    leftBtnCmnd1.WhenPressed(new StepShooterAngle(-10));
    upBtnCmnd1.WhenPressed(new ShootBall());
    downBtnCmnd1.WhenPressed(new ToggleGate());

    // case 1: LOADER mode
    rightBtnCmnd2.WhenPressed(new StepLoaderAngle(5));
    leftBtnCmnd2.WhenPressed(new StepLoaderAngle(-5));
    upBtnCmnd2.WhenPressed(new LoadBall());
    downBtnCmnd2.WhenPressed(new ToggleRollers());

}

Joystick* OI::GetJoystick() {
	return joy;
}
