/*
 * InitHolder.h
 *
 *  Created on: Mar 29, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_EXECHOLDER_H_
#define SRC_COMMANDS_EXECHOLDER_H_

#include <Commands/Command.h>

class ExecHolder: public Command {
public:
	ExecHolder();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif /* SRC_COMMANDS_EXECHOLDER_H_ */
