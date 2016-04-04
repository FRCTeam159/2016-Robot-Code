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
	enum {
		LOAD,LOW,CANCEL
	};
	int state;
	double elapsed_time;
	void SetLow();
	void SetLoad();
public:
	LoadBall();
	void Initialize();
	void Execute() {}
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* SRC_COMMANDS_LOADBALL_H_ */
