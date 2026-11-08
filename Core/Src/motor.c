	/*
 * motor.c
 *
 *  Created on: 7 Nov 2026
 *      Author: bpe82
 */
#include <main.h>
#include "motor.h"

#define PWM_MAX            999
#define LEFT_INVERT        0     /* 1 = flip left motor direction  */
#define RIGHT_INVERT       1     /* 1 = flip right motor direction */

extern TIM_HandleTypeDef htim2;


/* One motor = two PWM channels (one per IN pin). spd: -999..999 */
static void setMotor(uint32_t chA, uint32_t chB, int spd)
{
    if (spd >  PWM_MAX) spd =  PWM_MAX;
    if (spd < -PWM_MAX) spd = -PWM_MAX;

    if (spd > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, chA, spd);
        __HAL_TIM_SET_COMPARE(&htim2, chB, 0);
    }
    else if (spd < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, chA, 0);
        __HAL_TIM_SET_COMPARE(&htim2, chB, -spd);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2, chA, 0);
        __HAL_TIM_SET_COMPARE(&htim2, chB, 0);
    }
}

void setLeft(int spd)
{
    if (LEFT_INVERT) spd = -spd;
    setMotor(TIM_CHANNEL_1, TIM_CHANNEL_2, spd);   /* PA0 / PA1 -> IN1 / IN2 */
}

void setRight(int spd)
{
    if (RIGHT_INVERT) spd = -spd;
    setMotor(TIM_CHANNEL_3, TIM_CHANNEL_4, spd);   /* PA2 / PA3 -> IN3 / IN4 */
}

void forward(int s)   { setLeft(s);  setRight(s);  }
void backward(int s)  { setLeft(-s); setRight(-s); }
void turnLeft(int s)  { setLeft(-s); setRight(s);  }   /* spin in place */
void turnRight(int s) { setLeft(s);  setRight(-s); }   /* spin in place */
void stopMotors(void) { setLeft(0);  setRight(0);  }
