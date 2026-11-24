/*
 * mission.h
 *
 *  Created on: 11 Nov 2026
 *      Author: bpe82
 */

#ifndef INC_MISSION_H_
#define INC_MISSION_H_

#include <stdint.h>


void     missionStart(void);              /* press M: reset everything and begin the script */
uint8_t  missionTick(int speed);          /* call every loop in auto mode: 1 = running, 0 = finished or failed (robot stopped) */

extern uint8_t cornersSeen;


#endif /* INC_MISSION_H_ */
