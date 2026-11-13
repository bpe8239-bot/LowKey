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

/* Time-based backup for the branch count. id: 'A' = time to the first branch, 'B' = time between
 * branches, 'W' = +/- window. All in ms. Set over Bluetooth with KA / KB / KW. */
void     missionSetParam(char id, uint32_t value);
uint32_t missionGetParam(char id);

#endif /* INC_MISSION_H_ */
