/*
 * LoadBall.h
 *
 *  Created on: Mar 22, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_LOADBALL_H_
#define SRC_COMMANDS_LOADBALL_H_

#include <Commands/Command.h>

class LoadBall: public Command {
	int state;
	double elapsed_time;
	void FindLow();
	void SetLow();
	void SetMedium();
	void SetHigh();
public:
	LoadBall();
	void Initialize();
	void Execute() {}
	bool IsFinished();
	void End();
	void Interrupted() {End();}
};

#endif /* SRC_COMMANDS_LOADBALL_H_ */
