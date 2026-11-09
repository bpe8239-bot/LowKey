/*
 * line_following.h
 *
 *  Created on: 8 Nov 2026
 *      Author: bpe82
 */

#ifndef INC_LINE_FOLLOWING_H_
#define INC_LINE_FOLLOWING_H_

#include <stdint.h>
void lineFollow(int speed);
void monitorIR(void); //remove during final product
void lineFollowReset(void);
uint8_t pidTuneKey(char c);

#endif /* INC_LINE_FOLLOWING_H_ */
