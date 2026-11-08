/*
 * servo_claw.c
 *
 *  Created on: 7 Nov 2026
 *      Author: bpe82
 */

#include <main.h>
#include "servo_claw.h"

/* Private pointer to the timer used by the claw (TIM3 in this project). */
static TIM_HandleTypeDef *claw_htim = NULL;

HAL_StatusTypeDef Claw_Init(TIM_HandleTypeDef *htim)
{
    if (htim == NULL)
    {
        return HAL_ERROR;
    }

    /* Set a neutral pulse before PWM starts to avoid initially outputting 0. */
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, CLAW_NEUTRAL_PULSE_US);

    if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    claw_htim = htim;
    return HAL_OK;
}

HAL_StatusTypeDef Claw_SetPulseUs(uint16_t pulse_us)
{
    if (claw_htim == NULL ||
        pulse_us < CLAW_MIN_PULSE_US ||
        pulse_us > CLAW_MAX_PULSE_US)
    {
        return HAL_ERROR;
    }

    __HAL_TIM_SET_COMPARE(claw_htim, TIM_CHANNEL_2, pulse_us);
    return HAL_OK;
}

HAL_StatusTypeDef Claw_Open(void)
{
    return Claw_SetPulseUs(CLAW_OPEN_PULSE_US);
}

HAL_StatusTypeDef Claw_Close(void)
{
    return Claw_SetPulseUs(CLAW_CLOSE_PULSE_US);
}
