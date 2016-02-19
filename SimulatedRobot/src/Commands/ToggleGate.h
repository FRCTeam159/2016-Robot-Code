/*
 * OpenGate.h
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_TOGGLEGATE_H_
#define SRC_COMMANDS_TOGGLEGATE_H_

#include <Commands/Command.h>

class ToggleGate: public Command {
public:
	ToggleGate();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* SRC_COMMANDS_TOGGLEGATE_H_ */