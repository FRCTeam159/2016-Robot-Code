/*
 * ShootBall.h
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_SHOOTBALL_H_
#define SRC_COMMANDS_SHOOTBALL_H_

#include <Commands/Command.h>

class ShootBall: public Command {
	enum {
		FLYWHEELS_ON=1,
		PUSHER_ON=2,
	};
	int state;
public:
	ShootBall();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();

};

#endif /* SRC_COMMANDS_SHOOTBALL_H_ */
