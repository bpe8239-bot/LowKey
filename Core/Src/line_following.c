/*
 * line_following.c
 *
 *  Created on: 8 Nov 2026
 *      Author: bpe82
 */

#include "main.h"
#include "line_following.h"
#include "motor.h"
#include "bluetooth.h"
#include <stdio.h>

#define IR_REPORT_MS       400U  /* send IR report at most 2.5 times/sec */
#define LINE_LEVEL   GPIO_PIN_SET   /* level your sensor outputs when it sees the BLACK line.
                                       Flip to GPIO_PIN_RESET if it's the opposite */
#define IR_DEBUG     1              /* 1 = send sensor states to your phone every 400 ms */
#define RECOVERY_TIMEOUT_MS 700U
#define RECOVERY_MIN_SPEED 300

static int lastDir = 0;                /* -1 = left; 1 = right */
static uint32_t lostStart = 0;
static uint8_t recovering = 0;

//Read IR Sensor
static inline uint8_t irRead(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == LINE_LEVEL) ? 1U : 0U;
}

//Send sensor status through bluetooth
static void btReportIR(uint8_t l, uint8_t c, uint8_t r, const char *action)
{
#if IR_DEBUG
    static uint32_t lastReport = 0;
    uint32_t now = HAL_GetTick();
    if (now - lastReport >= IR_REPORT_MS)
    {
        char line[75];
        snprintf(line, sizeof(line), "[IR] L:%u C:%u R:%u | %s\r\n",
                 (unsigned)l, (unsigned)c, (unsigned)r, action);
        btPrint(line);
        lastReport = now;
    }
#else
    (void)l; (void)c; (void)r; (void)action;
#endif
}

//Monitor IR sensor
void monitorIR(void)
{
    const uint8_t l = irRead(GPIOB, GPIO_PIN_12);
    const uint8_t c = irRead(GPIOB, GPIO_PIN_13);
    const uint8_t r = irRead(GPIOB, GPIO_PIN_14);
    stopMotors();
    btReportIR(l, c, r, "MONITOR ONLY");
}

// Reset remembered direction
void lineFollowReset(void)
{
    lastDir = 0;
    lostStart = 0;
    recovering = 0;
}

//Line Following Auto mode
void lineFollow(int speed)
{
    const uint8_t l = irRead(GPIOB, GPIO_PIN_12);
    const uint8_t c = irRead(GPIOB, GPIO_PIN_13);
    const uint8_t r = irRead(GPIOB, GPIO_PIN_14);
    const char *action;

    if (l && !c && r)
    {
        stopMotors();
        btReportIR(l, c, r, "AMBIGUOUS - STOP");
        return;
    }

    if (l || c || r)
    {
        recovering = 0;
    }

    if (!l && c && !r) {
        forward(speed); action = "FORWARD";
    }
    else if (l && c && !r) {
        setLeft(speed / 3); setRight(speed); lastDir = -1; action = "LEFT";
    }
    else if (!l && c && r) {
        setLeft(speed); setRight(speed / 3); lastDir = 1; action = "RIGHT";
    }
    else if (l && !c && !r) {
        turnLeft(speed); lastDir = -1; action = "HARD LEFT";
    }
    else if (!l && !c && r) {
        turnRight(speed); lastDir = 1; action = "HARD RIGHT";
    }
    else if (l && c && r) {
        forward(speed); action = "CROSSING";
    }
    else
    {
     // All three IR sensors detect no line (000)
   	    if (!recovering)
   	    {
   	        lostStart = HAL_GetTick();
   	        recovering = 1;
   	    }

        if (lastDir == 0)
   	    {
   	        stopMotors();
   	        action = "NO LAST DIRECTION";
   	    }
        else if (HAL_GetTick() - lostStart >= RECOVERY_TIMEOUT_MS)
        {
   	        stopMotors();
   	        action = "RECOVERY FAILED";
   	    }
   	    else
   	    {
            int recoverySpeed = speed / 2;

    	     if (recoverySpeed < RECOVERY_MIN_SPEED)
            {
   	            recoverySpeed = RECOVERY_MIN_SPEED;
   	        }

   	        if (lastDir == -1)
   	        {
   	        	turnLeft(recoverySpeed);
    	        action = "RECOVER LEFT";
    	    }
            else
   	        {
   	            turnRight(recoverySpeed);
   	            action = "RECOVER RIGHT";
   	        }
   	    }
    }
    btReportIR(l, c, r, action);
}
