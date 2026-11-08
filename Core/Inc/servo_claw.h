/*
 * servo_claw.h
 *
 *  Created on: 7 Nov 2026
 *      Author: bpe82
 */

#ifndef INC_SERVO_CLAW_H_
#define INC_SERVO_CLAW_H_

#ifndef SERVO_CLAW_H
#define SERVO_CLAW_H

#include "stm32f1xx_hal.h"


/* Assumes TIM3 counter increments every 1 microsecond. */
#define CLAW_OPEN_PULSE_US       1650U
#define CLAW_CLOSE_PULSE_US      1300U
#define CLAW_NEUTRAL_PULSE_US    1550U


#define CLAW_MIN_PULSE_US        1000U
#define CLAW_MAX_PULSE_US        2000U

/* Call after MX_TIM3_Init(). This starts PWM on TIM3 channel 2 (PA7).
 * The caller passes &htim3, so this module does not depend on a global
 * timer-handle definition or a generated tim.h file. */
HAL_StatusTypeDef Claw_Init(TIM_HandleTypeDef *htim);

/* Set the high-pulse duration in microseconds (requires 1 MHz timer tick). */
HAL_StatusTypeDef Claw_SetPulseUs(uint16_t pulse_us);

HAL_StatusTypeDef Claw_Open(void);
HAL_StatusTypeDef Claw_Close(void);

#endif /* SERVO_CLAW_H */



#endif /* INC_SERVO_CLAW_H_ */
