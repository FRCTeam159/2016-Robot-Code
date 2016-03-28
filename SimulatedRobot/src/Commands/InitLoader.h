/*
 * LoaderGotoZero.h
 *
 *  Created on: Mar 23, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_INITLOADER_H_
#define SRC_COMMANDS_INITLOADER_H_

#include <Commands/Command.h>

class InitLoader: public Command {
public:
	InitLoader();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted() {End();}
};

#endif /* SRC_COMMANDS_INITLOADER_H_ */
