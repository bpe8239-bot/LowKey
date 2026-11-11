/*
 * line_following.c
 *
 *  Created on: 8 Nov 2026
 *      Author: bpe82
 *
 * 5-sensor line following with PATTERN-based junction recognition.
 *
 * Sensors, left to right:  S0 PB11   S1 PB12   S2 PB13   S3 PB14   S4 PB15
 * (S0/S4 are 4.5 cm from the centre, S1/S3 are 1.9 cm from the centre, line is 1.9 cm wide)
 *
 * Steering (OUTER_JUNCTION_ONLY = 1, default): PD uses S1, S2, S3 only (weights -2, 0, +2).
 * The outer sensors S0 / S4 are used only to recognise bars.
 *
 * Patterns (1 = black), left to right:
 *   00100 01100 00110   normal steering
 *   111xx (11100/11110) LEFT BAR   (branch or corner on the left)
 *   xx111 (00111/01111) RIGHT BAR
 *   1xxx1 (S0 and S4)   WIDE BAR   (crossing / end bar)
 *   10000 11000         NOT a bar: the line is drifting out on the left, pivot back to it
 *   00001 00011         same on the right
 *   10100 and similar   AMBIGUOUS: keep the last motion briefly, then stop
 *
 * After a bar:
 *   corner : everything goes white -> straight on for the corner distance, then pivot toward the bar side
 *   branch : the line is still there -> carries on (the bar-side steering sensor was ignored while it passed)
 *
 * Robot geometry: the sensor row is about 12 cm ahead of the drive-wheel axle, so
 *   corner distance  ~ 11 cm  (row has just passed the corner, axle must reach it)
 *   branch distance  ~ 13 cm  (row has just reached the branch bar, axle must reach its centre)
 *   -> time = distance / robot speed. Defaults below assume about 20 cm/s. Tune with KC / KJ.
 */

#include "main.h"
#include "line_following.h"
#include "mission.h"
#include "motor.h"
#include "bluetooth.h"
#include <stdio.h>

#define IR_COUNT           5
#define IR_REPORT_MS       400U  /* send IR report at most 2.5 times/sec */
#define LINE_LEVEL   GPIO_PIN_SET   /* level a sensor outputs when it sees the BLACK line.
                                       Flip to GPIO_PIN_RESET if it's the opposite */
#define IR_DEBUG     0              /* 1 = send sensor states while line following (blocks the loop
                                       ~30-45 ms per print). The pattern logger (L key) is better. */
#define TUNING_KEYS  1              /* 1 = live tuning enabled, 0 = disabled */

#define OUTER_JUNCTION_ONLY 1       /* 1 = S0/S4 never steer, PD uses S1-S3
                                       0 = all five sensors steer (assumes equal sensor spacing) */

// PD controller settings (copy your tuned values here when done)
#if OUTER_JUNCTION_ONLY
#define KP_DEFAULT         350.0f   // PD over 3 sensors
#define HARD_ERROR         2.0f     // |error| at or beyond this: pivot in place (line only under S1 or S3)
#else
#define KP_DEFAULT         300.0f   // PD over 5 sensors steers in finer steps, so a lower Kp
#define HARD_ERROR         3.0f
#endif
#define KD_DEFAULT         0.0f     //Damper
#define CONTROL_PERIOD_MS  10U
#define PWM_MAX            999

// Dashed line gap + 90 degree corner
#define GAP_CROSS_MS       200U     // straight-ahead hold when the line was centred (dashed gap)
#define CORNER_FWD_MS      550U     // straight-ahead time before pivoting at a corner (~11 cm at 20 cm/s) (tune: KC<ms>;)
#define PIVOT_TIMEOUT_MS   1500U    // give up if the pivot finds no line
#define RECOVERY_MIN_SPEED 300      // slowest pivot speed

// Side bars (branches / corners)
#define BAR_LOCKOUT_MS     250U     // ignore a new bar event this soon after the last one
#define MASK_EXTRA_MS      60U      // keep ignoring the bar-side sensor this long after the bar ends

// Ambiguous patterns (e.g. while sweeping across lines during a sharp turn)
#define AMBIG_HOLD_MS      100U     // keep the last motor command this long before stopping

// Side branch (T-junction) turn, used by mission.c
#define BRANCH_FWD_MS      650U     // straight-ahead time from seeing the branch to starting the pivot (~13 cm at 20 cm/s) (tune: KJ<ms>;)

// Pattern logger
#define LOG_SIZE           160U     // entries kept (newest overwrite the oldest)
#define LOG_STABLE_MS      5U       // a pattern must hold this long to be recorded (filters flicker)

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} IrPin_t;

static const IrPin_t  irPin[IR_COUNT]    = { {GPIOB, GPIO_PIN_11}, {GPIOB, GPIO_PIN_12}, {GPIOB, GPIO_PIN_13},
                                             {GPIOB, GPIO_PIN_14}, {GPIOB, GPIO_PIN_15} };
static const int8_t   irWeight[IR_COUNT] = { -4, -2, 0, 2, 4 };

static float kp = KP_DEFAULT;
static float kd = KD_DEFAULT;
static uint32_t cornerFwdMs = CORNER_FWD_MS;
static uint32_t branchFwdMs = BRANCH_FWD_MS;

static int lastDir = 0;                /* -1 = left; 1 = right; 0 = centred / unknown */
static uint8_t holdOk = 0;             /* 1 = line was centred or on a gentle edge, keep going straight first */
static uint32_t lostStart = 0;
static uint8_t recovering = 0;

// Ambiguous pattern timer
static uint8_t  ambiguous = 0;
static uint32_t ambigStart = 0;

// Corner counting
static uint8_t cornerCount = 0;        /* corners turned so far */
static uint8_t cornerPivoting = 0;     /* 1 = currently pivoting at a corner */

// Side bar tracking
static int8_t  pendingBar = 0;         /* event waiting for mission.c: -1 left, +1 right */
static int8_t  prevBarNow = 0;
static int8_t  barLatch = 0;           /* side of the bar we are crossing / just crossed (kept through an all-white corner) */
static uint32_t lastBarEvent = 0;
static uint32_t lastBarOn = 0;

// Controller memory
static float previousError = 0.0f;
static uint8_t previousValid = 0;
static uint32_t lastControlMs = 0;

// Pattern logger memory
static uint32_t logTime[LOG_SIZE];
static uint8_t  logPat[LOG_SIZE];
static uint16_t logHead = 0;           /* next slot to write */
static uint16_t logCount = 0;          /* entries stored (max LOG_SIZE) */
static uint8_t  logCand = 0xFF;
static uint32_t logCandSince = 0;
static uint8_t  logLast = 0xFF;

//Read IR Sensor. Return 1 if black is detected
static inline uint8_t irRead(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == LINE_LEVEL) ? 1U : 0U;
}

// Raw sensors as bits: bit0 = far left ... bit4 = far right
uint8_t lineReadBits(void)
{
    uint8_t bits = 0;
    for (uint8_t i = 0; i < IR_COUNT; i++)
    {
        if (irRead(irPin[i].port, irPin[i].pin))
            bits |= (uint8_t)(1U << i);
    }
    return bits;
}

// Returns and clears the pending side-bar event
int8_t lineBarEvent(void)
{
    int8_t e = pendingBar;
    pendingBar = 0;
    return e;
}

uint8_t  lineFollowCorners(void)      { return cornerCount; }
uint32_t lineFollowBranchFwdMs(void)  { return branchFwdMs; }

/* ---------------- Pattern logger ---------------- */

void lineLogClear(void)
{
    logHead = 0;
    logCount = 0;
    logCand = 0xFF;
    logLast = 0xFF;
}

// Call every main-loop pass: records a pattern once it has been stable for LOG_STABLE_MS
void lineLogTick(void)
{
    const uint8_t p = lineReadBits();
    const uint32_t now = HAL_GetTick();

    if (p != logCand)
    {
        logCand = p;
        logCandSince = now;
        return;
    }

    if (p != logLast && (now - logCandSince) >= LOG_STABLE_MS)
    {
        logLast = p;
        logTime[logHead] = logCandSince;
        logPat[logHead] = p;
        logHead = (uint16_t)((logHead + 1U) % LOG_SIZE);
        if (logCount < LOG_SIZE) logCount++;
    }
}

// Prints the log, oldest first. Each line: pattern (S0..S4) and ms since the previous entry.
// Stop the robot first: sending takes about 2 seconds for a full log.
void lineLogDump(void)
{
    char line[64];
    uint16_t idx = (uint16_t)((logHead + LOG_SIZE - logCount) % LOG_SIZE);
    uint32_t prev = logTime[idx];

    snprintf(line, sizeof(line), "[LOG] %u entries (pattern S0..S4, +ms)\r\n", (unsigned)logCount);
    btPrint(line);

    for (uint16_t n = 0; n < logCount; n++)
    {
        char pat[IR_COUNT + 1];
        for (uint8_t i = 0; i < IR_COUNT; i++)
            pat[i] = ((logPat[idx] >> i) & 1U) ? '1' : '0';
        pat[IR_COUNT] = '\0';

        snprintf(line, sizeof(line), "%s +%lu\r\n", pat, (unsigned long)(logTime[idx] - prev));
        btPrint(line);

        prev = logTime[idx];
        idx = (uint16_t)((idx + 1U) % LOG_SIZE);
    }

    btPrint("[LOG] end\r\n");
    lineLogClear();
}

//Send sensor status through bluetooth (only when IR_DEBUG is 1)
static void btReportIR(uint8_t bits, const char *action)
{
#if IR_DEBUG
    static uint32_t lastReport = 0;
    uint32_t now = HAL_GetTick();
    if (now - lastReport >= IR_REPORT_MS)
    {
        char pat[IR_COUNT + 1];
        char line[80];
        for (uint8_t i = 0; i < IR_COUNT; i++)
            pat[i] = ((bits >> i) & 1U) ? '1' : '0';
        pat[IR_COUNT] = '\0';

        snprintf(line, sizeof(line), "[IR] %s | %s\r\n", pat, action);
        btPrint(line);
        lastReport = now;
    }
#else
    (void)bits; (void)action;
#endif
}

// Keep motor command within forward PWM range
static int clampSpeed(int value)
{
    if (value > PWM_MAX) return PWM_MAX;
    if (value < 0) return 0;

    return value;
}

// Active sensors with a gap between them (e.g. 10100) can't be trusted
static uint8_t isAmbiguous(const uint8_t s[IR_COUNT])
{
    int first = -1, last = -1;
    for (int i = 0; i < IR_COUNT; i++)
    {
        if (s[i])
        {
            if (first < 0) first = i;
            last = i;
        }
    }
    if (first < 0) return 0;
    for (int i = first; i <= last; i++)
    {
        if (!s[i]) return 1;
    }
    return 0;
}

// Reset controller when entering auto mode
void lineFollowReset(void)
{
    lastDir = 0;
    holdOk = 0;
    lostStart = 0;
    recovering = 0;
    ambiguous = 0;
    ambigStart = 0;
    cornerCount = 0;
    cornerPivoting = 0;
    pendingBar = 0;
    prevBarNow = 0;
    barLatch = 0;
    lastBarEvent = 0;
    lastBarOn = 0;
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

    const uint8_t raw = lineReadBits();
    uint8_t s[IR_COUNT];
    for (uint8_t i = 0; i < IR_COUNT; i++)
        s[i] = (uint8_t)((raw >> i) & 1U);

    /* ---- Side bar detection by PATTERN ---- */
#if OUTER_JUNCTION_ONLY
    /* a bar is three sensors in a row reaching the centre, not one outer sensor.
     *   left bar  = S0 S1 S2 on  (11100 / 11110)
     *   right bar = S4 S3 S2 on  (00111 / 01111)
     * 10000 or 11000 (no centre sensor) is the line drifting out, not a bar. */
    const uint8_t leftBar  = (s[0] && s[1] && s[2]) ? 1U : 0U;
    const uint8_t rightBar = (s[4] && s[3] && s[2]) ? 1U : 0U;
#else
    /* all sensors steer, so a bar is a far sensor AND the centre sensor on together */
    const uint8_t leftBar  = (s[0] && s[2]) ? 1U : 0U;
    const uint8_t rightBar = (s[4] && s[2]) ? 1U : 0U;
#endif
    int8_t barNow = 0;                       /* -1 left bar, +1 right bar, 2 both (wide), 0 none */
    if (leftBar && rightBar) barNow = 2;
    else if (leftBar)        barNow = -1;
    else if (rightBar)       barNow = 1;

    if (barNow == -1 || barNow == 1)
    {
        if (prevBarNow == 0 && (now - lastBarEvent) >= BAR_LOCKOUT_MS)
        {
            pendingBar = barNow;             /* report a new bar to mission.c */
            lastBarEvent = now;
        }
        barLatch = barNow;
        lastBarOn = now;
    }
    else if (barNow == 2)
    {
        barLatch = 0;
    }
    prevBarNow = barNow;

#if OUTER_JUNCTION_ONLY
    s[0] = 0;                                /* outer sensors never steer */
    s[4] = 0;
#endif

    /* ---- Ignore the steering sensor on the bar side while the bar passes ---- */
    uint8_t maskActive = 0;
    if (barLatch != 0 && (barNow == barLatch || (now - lastBarOn) < MASK_EXTRA_MS))
        maskActive = 1;

    if (maskActive)
    {
#if OUTER_JUNCTION_ONLY
        if (barLatch < 0) { s[1] = 0; }      /* only S1 / S3 steer on each side */
        else              { s[3] = 0; }
#else
        if (barLatch < 0) { s[0] = 0; s[1] = 0; }
        else              { s[3] = 0; s[4] = 0; }
#endif
    }

    uint8_t count = 0;
    for (uint8_t i = 0; i < IR_COUNT; i++)
        count += s[i];

    /* Bar is behind us and a sensor sees the line again: forget the bar side */
    if (barLatch != 0 && !maskActive && count > 0)
        barLatch = 0;

    /* While a bar is latched, steering memory points at the bar side (used by the corner pivot) */
    if (barLatch != 0)
    {
        lastDir = barLatch;
        holdOk = 1;
    }

    /* ambiguity is checked once, and its timer clears as soon as the pattern makes sense again */
    const uint8_t amb = (count > 0) ? isAmbiguous(s) : 0U;
    if (!amb) ambiguous = 0;

    const char *action = "PD";

    // Any sensor sees the line again: leave recovery / corner
    if (count > 0)
    {
        if (cornerPivoting)
        {
            cornerCount++;
            cornerPivoting = 0;
        }
        recovering = 0;
    }

    // Wide bar = both outer sensors on (crossing or end bar)
    if ((raw & 0x11U) == 0x11U)
    {
        forward(speed);
        previousValid = 0;
        lastDir = 0;
        holdOk = 1;
        action = "CROSSING";
    }

    // All white: dashed gap, 90 degree corner, or lost line
    else if (count == 0)
    {
        previousValid = 0;

        if (!recovering)
        {
            lostStart = now;
            recovering = 1;
        }

        /* How long to keep going straight before pivoting:
         *   line was centred             -> short hold (dashed gap)
         *   a side bar was just crossed  -> corner distance, then pivot toward the bar side
         *   line was lost sideways       -> no hold, pivot right away */
        uint32_t holdMs = 0;
        if (holdOk)
            holdMs = (lastDir == 0) ? GAP_CROSS_MS : cornerFwdMs;

        uint32_t lostMs = now - lostStart;

        if (lostMs < holdMs)
        {
            forward(speed);
            action = (lastDir == 0) ? "GAP - HOLD" : "CORNER - FORWARD";
        }
        else if (lastDir == 0)
        {
            stopMotors();
            action = "NO LAST DIRECTION";
        }
        else if ((lostMs - holdMs) >= PIVOT_TIMEOUT_MS)
        {
            stopMotors();
            action = "PIVOT FAILED";
        }
        else
        {
            int pivotSpeed = speed / 2;
            if (pivotSpeed < RECOVERY_MIN_SPEED)
                pivotSpeed = RECOVERY_MIN_SPEED;

            if (holdOk)                 // only a real corner pivot counts, not a sideways recovery
                cornerPivoting = 1;

            if (lastDir == -1)
            {
                turnLeft(pivotSpeed);
                action = "CORNER - PIVOT LEFT";
            }
            else
            {
                turnRight(pivotSpeed);
                action = "CORNER - PIVOT RIGHT";
            }
        }
    }

    // Ambiguous pattern: keep the last motor command briefly (sharp turns sweep across lines), then stop
    else if (amb)
    {
        previousValid = 0;
        if (!ambiguous)
        {
            ambiguous = 1;
            ambigStart = now;
        }

        if ((now - ambigStart) >= AMBIG_HOLD_MS)
        {
            stopMotors();
            action = "AMBIGUOUS - STOP";
        }
        else
        {
            action = "AMBIGUOUS - HOLD";
        }
    }

    // Line found: steer
    else
    {
        float sum = 0.0f;
        for (uint8_t i = 0; i < IR_COUNT; i++)
            sum += (float)irWeight[i] * (float)s[i];

        const float error = sum / (float)count;      /* negative = line on the left */

        // Line far off to the left: pivot left
        if (error <= -HARD_ERROR)
        {
            turnLeft(speed);
            lastDir = -1;
            holdOk = 0;
            previousValid = 0;
            action = "HARD LEFT";
        }
        // Line far off to the right: pivot right
        else if (error >= HARD_ERROR)
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

            if (barLatch != 0)
            {
                lastDir = barLatch;              /* keep pointing at the bar side */
                action = "PD (BAR)";
            }
            else if (error < 0.0f)
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
    }

    btReportIR(raw, action);
}

/* ---------------------------------------------------------------------------
 * Live tuning over Bluetooth (end each entry with ; or Enter)
 *
 *   T            shows all tuned values immediately
 *   KP350;       Kp = 350
 *   KD0.35;      Kd = 0.35
 *   KC550;       corner straight-ahead time, ms
 *   KJ650;       branch straight-ahead time, ms
 *   KA400;       time from the end of the 2nd corner to the first branch, ms   (mission backup)
 *   KB1000;      time between branches, ms                                      (mission backup)
 *   KW350;       +/- window around the expected branch time, ms                 (mission backup)
 *   K;           shows the values
 *
 * pidTuneKey() must be the FIRST thing called in handleBtCommand() in main.c.
 * ------------------------------------------------------------------------- */
#if TUNING_KEYS
#define ENTRY_TIMEOUT_MS  5000U
#define KP_MAX            2000.0f
#define KD_MAX            10.0f
#define MS_MAX            5000.0f

static uint8_t  entryState = 0;      /* 0 idle, 1 got K, 2 reading value */
static char     entryTarget = 0;     /* 'P', 'D', 'C', 'J', 'A', 'B' or 'W' */
static char     entryBuf[10];
static uint8_t  entryLen = 0;
static uint32_t entryTick = 0;

static void printGains(void)
{
    char line[110];
    /* print without %f (newlib-nano has no float printf by default) */
    int kdHundredths = (int)(kd * 100.0f + 0.5f);
    snprintf(line, sizeof(line),
             "[TUNE] Kp=%d Kd=%d.%02d Corner=%lu Branch=%lu T1=%lu Ts=%lu Win=%lu ms\r\n",
             (int)(kp + 0.5f), kdHundredths / 100, kdHundredths % 100,
             (unsigned long)cornerFwdMs, (unsigned long)branchFwdMs,
             (unsigned long)missionGetParam('A'), (unsigned long)missionGetParam('B'),
             (unsigned long)missionGetParam('W'));
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

/* Returns 1 if the character was consumed by the tuning entry.
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
        if (c == 'T' || c == 't')          /* show values instantly */
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

    if (entryState == 1)                       /* after K: expect P, D, C, J, A, B or W */
    {
        if (isEnd) { entryState = 0; printGains(); return 1; }

        char up = c;
        if (up >= 'a' && up <= 'z') up = (char)(up - 32);

        if (up == 'P' || up == 'D' || up == 'C' || up == 'J' || up == 'A' || up == 'B' || up == 'W')
        {
            entryTarget = up;
        }
        else
        {
            entryState = 0;
            btPrint("[TUNE] KP KD KC KJ KA KB KW then a number and ;\r\n");
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
        if (!parseValue(entryBuf, &v))   { btPrint("[TUNE] Invalid number\r\n"); return 1; }

        if (entryTarget == 'P')      kp = (v > KP_MAX) ? KP_MAX : v;
        else if (entryTarget == 'D') kd = (v > KD_MAX) ? KD_MAX : v;
        else if (entryTarget == 'C') cornerFwdMs = (uint32_t)((v > MS_MAX) ? MS_MAX : v);
        else if (entryTarget == 'J') branchFwdMs = (uint32_t)((v > MS_MAX) ? MS_MAX : v);
        else                         missionSetParam(entryTarget, (uint32_t)((v > MS_MAX) ? MS_MAX : v));

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
        btPrint("[TUNE] Value too long\r\n");
    }
    return 1;
#else
    (void)c;
    return 0;
#endif
}
