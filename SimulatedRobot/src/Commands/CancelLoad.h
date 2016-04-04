/*
 * ExpelBall.h
 *
 *  Created on: Mar 31, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_CANCELLOAD_H_
#define SRC_COMMANDS_CANCELLOAD_H_

#include <Commands/Command.h>

class CancelLoad: public Command {
public:
	CancelLoad();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* SRC_COMMANDS_CANCELLOAD_H_ */
