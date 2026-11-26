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

/* Rear alignment sensor */
uint8_t  lineReadRear(void);               /* 1 = rear sensor sees the line */
uint8_t  lineAligned(void);                /* 1 = front centre sensor AND rear sensor both on the line */

extern uint32_t cornerSpeed;
extern uint32_t branchRunSpeed;
extern uint8_t is_gap;

#endif /* INC_LINE_FOLLOWING_H_ */
