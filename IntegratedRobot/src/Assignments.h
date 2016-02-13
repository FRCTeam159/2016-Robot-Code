/*
 * Assignments.h
 *
 *  Created on: Feb 13, 2016
 *      Author: 159
 */

#ifndef SRC_ASSIGNMENTS_H_
#define SRC_ASSIGNMENTS_H_

#ifndef SIMULATION
#define CANTALON_PUSHER
#define CANTALON_GATE
#endif

enum can_ids {
	CAN_HOLDER_GATE = 1,
	CAN_HOLDER_PUSHER =2
};

enum pwm_ids {
	PWM_HOLDER_GATE = 1,
	PWM_HOLDER_PUSHER =2
};

enum dio_ids {
	REVGATELIMIT=1,
	FWDGATELIMIT=2,
	IRSENSOR=0
};

#ifdef CANTALON_PUSHER
#define HOLDER_PUSHER CAN_HOLDER_PUSHER
#else
#define HOLDER_PUSHER PWM_HOLDER_PUSHER
#endif

#ifdef CANTALON_GATE
#define HOLDER_GATE CAN_HOLDER_GATE
#else
#define HOLDER_GATE PWM_HOLDER_GATE
#endif

#endif
/* SRC_ASSIGNMENTS_H_ */