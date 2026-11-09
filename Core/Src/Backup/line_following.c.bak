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
#define IR_DEBUG     0              /* 1 = send sensor states while line following (blocks the loop
                                       ~30-45 ms per print). The I monitor mode always prints. */
#define TUNING_KEYS  1              /* 1 = Kp/Kd tuning enabled, 0 = disabled */

// PD controller settings (copy your tuned values here when done)
#define KP_DEFAULT         100.0f //Error correction rate
#define KD_DEFAULT         0.35f //Damper
#define CONTROL_PERIOD_MS  10U
#define PWM_MAX            999

// Dashed line gap + lost line recovery
#define GAP_CROSS_MS       200U     // drive straight this long when the line vanishes
#define RECOVERY_TIMEOUT_MS 2000U
#define RECOVERY_MIN_SPEED 300

static float kp = KP_DEFAULT;
static float kd = KD_DEFAULT;

static int lastDir = 0;                /* -1 = left; 1 = right; 0 = centred / unknown */
static uint8_t holdOk = 0;             /* 1 = last motion was straight/gentle, safe to hold across a gap */
static uint32_t lostStart = 0;
static uint8_t recovering = 0;

// Controller memory
static float previousError = 0.0f;
static uint8_t previousValid = 0;
static uint32_t lastControlMs = 0;

//Read IR Sensor. Return 1 if black is detected
static inline uint8_t irRead(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == LINE_LEVEL) ? 1U : 0U;
}

//Send sensor status through bluetooth (force = 1 prints even when IR_DEBUG is 0)
static void btReportIR(uint8_t l, uint8_t c, uint8_t r, const char *action, uint8_t force)
{
#if !IR_DEBUG
    if (!force) return;
#else
    (void)force;
#endif
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
}

//Monitor IR sensor (remove during final product)
void monitorIR(void)
{
    const uint8_t l = irRead(GPIOB, GPIO_PIN_12);
    const uint8_t c = irRead(GPIOB, GPIO_PIN_13);
    const uint8_t r = irRead(GPIOB, GPIO_PIN_14);
    stopMotors();
    btReportIR(l, c, r, "MONITOR ONLY", 1);
}

// Calculate weighted position error
static float calculateLineError(uint8_t l,
                                uint8_t c,
                                uint8_t r)
{
    int total = l + c + r;

    if (total == 0)
        return 0.0f;

    return (-2.0f * l + 0.0f * c + 2.0f * r)
           / total;
}

// Keep motor command within forward PWM range
static int clampSpeed(int value)
{
    if (value > PWM_MAX) return PWM_MAX;
    if (value < 0) return 0;

    return value;
}

// Reset controller when entering auto mode
void lineFollowReset(void)
{
    lastDir = 0;
    holdOk = 0;
    lostStart = 0;
    recovering = 0;
    previousError = 0.0f;
    previousValid = 0;
    lastControlMs = HAL_GetTick() - CONTROL_PERIOD_MS;
}

// Automatic line following with PD control
void lineFollow(int speed)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - lastControlMs;

    // Update controller approximately every 10 ms
    if (elapsed < CONTROL_PERIOD_MS)
        return;

    lastControlMs = now;

    uint8_t l = irRead(GPIOB, GPIO_PIN_12);
    uint8_t c = irRead(GPIOB, GPIO_PIN_13);
    uint8_t r = irRead(GPIOB, GPIO_PIN_14);

    const char *action = "PD";

    // Any sensor sees the line again: leave recovery
    if (l || c || r)
    {
        recovering = 0;
    }

    // Intersection / wide line
    if (l && c && r)
    {
        forward(speed);
        previousValid = 0;
        lastDir = 0;
        holdOk = 1;
        action = "CROSSING";
    }

    // All white: dashed gap or lost line
    else if (!l && !c && !r)
    {
        previousValid = 0;

        if (!recovering)
        {
            lostStart = now;
            recovering = 1;
        }

        if (holdOk && (now - lostStart) < GAP_CROSS_MS)
        {
            // Probably a dashed-line gap: drive straight through it
            forward(speed);
            action = "GAP - HOLD";
        }
        else if (lastDir == 0)
        {
            stopMotors();
            action = "NO LAST DIRECTION";
        }
        else if ((now - lostStart) >= RECOVERY_TIMEOUT_MS)
        {
            stopMotors();
            action = "RECOVERY FAILED";
        }
        else
        {
            int recoverySpeed = speed / 2;
            if (recoverySpeed < RECOVERY_MIN_SPEED)
                recoverySpeed = RECOVERY_MIN_SPEED;

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

    // Ambiguous sensor pattern
    else if (l && !c && r)
    {
        stopMotors();
        previousValid = 0;
        action = "AMBIGUOUS - STOP";
    }

    // Retain existing hard-left recovery
    else if (l && !c && !r)
    {
        turnLeft(speed);
        lastDir = -1;
        holdOk = 0;
        previousValid = 0;
        action = "HARD LEFT";
    }

    // Retain existing hard-right recovery
    else if (!l && !c && r)
    {
        turnRight(speed);
        lastDir = 1;
        holdOk = 0;
        previousValid = 0;
        action = "HARD RIGHT";
    }

    // Normal PD line following
    else
    {
        float error = calculateLineError(l, c, r);

        float dt = elapsed / 1000.0f;

        float derivative = 0.0f;

        if (previousValid)
        {
            derivative = (error - previousError) / dt;
        }

        float correction = kp * error + kd * derivative;

        int leftSpeed = clampSpeed(
            (int)(speed + correction));

        int rightSpeed = clampSpeed(
            (int)(speed - correction));

        setLeft(leftSpeed);
        setRight(rightSpeed);

        previousError = error;
        previousValid = 1;
        holdOk = 1;

        if (error < 0.0f)
        {
            lastDir = -1;
            action = "PD LEFT";
        }
        else if (error > 0.0f)
        {
            lastDir = 1;
            action = "PD RIGHT";
        }
        else
        {
            lastDir = 0;
            action = "PD CENTER";
        }
    }

    btReportIR(l, c, r, action, 0);
}

/* ---------------------------------------------------------------------------
 * Live Kp / Kd entry over Bluetooth
 *
 *   T            shows Kp and Kd immediately
 *   KP150;       sets Kp = 150      (end with ; or Enter)
 *   KD0.35;      sets Kd = 0.35     (end with ; or Enter)
 *   K;           shows Kp and Kd
 *
 * pidTuneKey() must be the FIRST thing called in handleBtCommand() in main.c.
 * ------------------------------------------------------------------------- */
#if TUNING_KEYS
#define ENTRY_TIMEOUT_MS  5000U
#define KP_MAX            2000.0f
#define KD_MAX            10.0f

static uint8_t  entryState = 0;      /* 0 idle, 1 got K, 2 reading value */
static char     entryTarget = 0;     /* 'P' or 'D' */
static char     entryBuf[10];
static uint8_t  entryLen = 0;
static uint32_t entryTick = 0;

static void printGains(void)
{
    char line[64];
    /* print without %f (newlib-nano has no float printf by default) */
    int kdHundredths = (int)(kd * 100.0f + 0.5f);
    snprintf(line, sizeof(line), "[PD] Kp=%d Kd=%d.%02d\r\n",
             (int)(kp + 0.5f), kdHundredths / 100, kdHundredths % 100);
    btPrint(line);
}

/* "150" or "0.35" -> float. Returns 1 if valid. */
static uint8_t parseValue(const char *s, float *out)
{
    int whole = 0;
    float frac = 0.0f, scale = 0.1f;
    uint8_t seenDot = 0, seenDigit = 0;

    for (; *s; s++)
    {
        if (*s == '.')
        {
            if (seenDot) return 0;
            seenDot = 1;
        }
        else if (*s >= '0' && *s <= '9')
        {
            seenDigit = 1;
            if (!seenDot) whole = whole * 10 + (*s - '0');
            else { frac += (*s - '0') * scale; scale *= 0.1f; }
        }
        else return 0;
    }
    if (!seenDigit) return 0;
    *out = (float)whole + frac;
    return 1;
}
#endif

/* Returns 1 if the character was consumed by Kp/Kd entry.
 * With TUNING_KEYS = 0 this always returns 0, so main.c can keep calling it. */
uint8_t pidTuneKey(char c)
{
#if TUNING_KEYS
    const uint8_t isEnd = (c == '\r' || c == '\n' || c == ';');

    /* abandon a half-typed entry after 5 s */
    if (entryState != 0 && (HAL_GetTick() - entryTick) > ENTRY_TIMEOUT_MS)
        entryState = 0;

    if (entryState == 0)
    {
        if (c == 'K' || c == 'k')
        {
            entryState = 1;
            entryTick = HAL_GetTick();
            return 1;
        }
        if (c == 'T' || c == 't')          /* show gains instantly */
        {
            printGains();
            return 1;
        }
        return 0;
    }

    /* stop commands always win, even mid-entry */
    if (c == 'X' || c == 'x' || c == ' ')
    {
        entryState = 0;
        return 0;
    }

    entryTick = HAL_GetTick();

    if (entryState == 1)                       /* after K: expect P or D */
    {
        if (isEnd) { entryState = 0; printGains(); return 1; }

        if (c == 'P' || c == 'p')      entryTarget = 'P';
        else if (c == 'D' || c == 'd') entryTarget = 'D';
        else
        {
            entryState = 0;
            btPrint("[PD] Use KP<value>; or KD<value>;\r\n");
            return 1;
        }
        entryState = 2;
        entryLen = 0;
        entryBuf[0] = '\0';
        return 1;
    }

    /* entryState == 2: collecting the number */
    if (isEnd)
    {
        float v;
        entryState = 0;

        if (entryLen == 0)               { printGains(); return 1; }
        if (!parseValue(entryBuf, &v))   { btPrint("[PD] Invalid number\r\n"); return 1; }

        if (entryTarget == 'P') kp = (v > KP_MAX) ? KP_MAX : v;
        else                    kd = (v > KD_MAX) ? KD_MAX : v;

        printGains();
        return 1;
    }

    if (entryLen < sizeof(entryBuf) - 1)
    {
        entryBuf[entryLen++] = c;
        entryBuf[entryLen] = '\0';
    }
    else
    {
        entryState = 0;
        btPrint("[PD] Value too long\r\n");
    }
    return 1;
#else
    (void)c;
    return 0;
#endif
}
