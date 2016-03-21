/*
 * Assignments.h
 *
 *  Created on: Mar 6, 2016
 *      Author: dean
 */

#ifndef SRC_ASSIGNMENTS_H_
#define SRC_ASSIGNMENTS_H_

enum motorIDs {  // PWM or CAN ids
	DRIVE_LEFT    = 1,
	DRIVE_RIGHT   = 2,
	HOLDER_GATE   = 3,
	HOLDER_PUSH   = 4,
	SHOOTER_ANGLE = 5,
	SHOOTER_LEFT  = 6,
	SHOOTER_RIGHT = 7,
};

enum dioIDs {  // limit switch ids
	GATE_MIN     = 0,
	GATE_MAX     = 1,
};

enum AnalogIDs {  // limit switch ids
	SHOOTER_PITCH  = 1, // gyro in simulation
	BALL_SENSOR    = 2, // sonar in simulation
	DRIVE_ANGLE    = 3, // gyro in simulation

};

#define WHEEL_DIAMETER 7.5
#define DEADBAND 0.25

#ifdef SIMULATION
#define WHEEL_TICKS 360 // default encoder ticks given in simulation
#define INVERT_RIGHT_SIDE true // set true if wheel axis point in same direction on both sides
#else // replace these values for whatever is appropriate for real drive-train
#define WHEEL_TICKS 360
#define INVERT_RIGHT_SIDE true
#endif

#define XBOX_GAMEPAD 1
#define EXTREME_3D   2

#ifdef SIMULATION
#define JOYTYPE XBOX_GAMEPAD
#else
#define JOYTYPE EXTREME_3D
#endif

#endif /* SRC_ASSIGNMENTS_H_ */
