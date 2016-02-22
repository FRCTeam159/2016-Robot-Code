/*
 * OI.cpp
 *
 *  Created on: Jun 3, 2014
 *      Author: alex
 */

#include "OI.h"
#include "Commands/ToggleGate.h"
#include "Commands/ShootBall.h"
#include "Commands/StepShooterAngle.h"


OI::OI() {

	joy= new Joystick(0);

    // Create some buttons
    JoystickButton* d_right = new JoystickButton(joy, 2);
    JoystickButton* d_left= new JoystickButton(joy, 3);
    JoystickButton* d_down= new JoystickButton(joy, 1);
    JoystickButton* d_up = new JoystickButton(joy, 4);

    // Connect the buttons to commands

    d_right->WhenPressed(new StepShooterAngle(1));
    d_left->WhenPressed(new StepShooterAngle(-1));

    d_down->ToggleWhenPressed(new ToggleGate());
    d_up->WhenPressed(new ShootBall());

}


Joystick* OI::GetJoystick() {
	return joy;
}
