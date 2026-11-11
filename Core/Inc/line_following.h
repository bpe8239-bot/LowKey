/*
 * line_following.h
 *
 *  Created on: 8 Nov 2026
 *      Author: bpe82
 */

#ifndef INC_LINE_FOLLOWING_H_
#define INC_LINE_FOLLOWING_H_

#include <stdint.h>

void     lineFollow(int speed);
void     lineFollowReset(void);
uint8_t  pidTuneKey(char c);               /* live tuning entry (KP / KD / KC / KJ / KA / KB / KW) and T */

/* Used by mission.c */
uint8_t  lineReadBits(void);               /* raw sensors, bit i = 1 when sensor i sees the line */
int8_t   lineBarEvent(void);               /* side bar seen: -1 = left, +1 = right, 0 = none (clears the event) */
uint8_t  lineFollowCorners(void);          /* corners turned since lineFollowReset() */
uint32_t lineFollowBranchFwdMs(void);      /* straight-ahead time before turning into a branch (tunable: KJ<ms>;) */

/* Sensor pattern logger: records every change of the 5-bit pattern with a timestamp */
void     lineLogTick(void);                /* call every main-loop pass */
void     lineLogClear(void);
void     lineLogDump(void);                /* prints the log over Bluetooth (oldest first), then clears it */

#endif /* INC_LINE_FOLLOWING_H_ */
