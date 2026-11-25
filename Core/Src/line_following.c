/*
 * line_following.c
 *
 *  Created on: 8 Nov 2026
 *      Author: bpe82
 *
 * 5-sensor line following with PATTERN-based junction recognition.
 *
 * Sensors, left to right:  S0 PB1  S1 PB0   S2 PB13   S3 PB14   S4 PB15
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
#include "oled.h"

#define IR_COUNT           5
#define IR_REPORT_MS       400U  /* send IR report at most 2.5 times/sec */
#define LINE_LEVEL   GPIO_PIN_SET   /* level a sensor outputs when it sees the BLACK line.
                                       Flip to GPIO_PIN_RESET if it's the opposite */
#define IR_DEBUG     1              /* 1 = send sensor states while line following (blocks the loop
                                       ~30-45 ms per print).*/
#define TUNING_KEYS  1              /* 1 = live tuning enabled, 0 = disabled*/

#define OUTER_JUNCTION_ONLY 0      /* 1 = S0/S4 never steer, PD uses S1-S3
                                       0 = all five sensors steer (assumes equal sensor spacing) */
//Rear IR sensor
#define REAR_IR_PORT       GPIOB
#define REAR_IR_PIN        GPIO_PIN_3

// PD controller settings (copy your tuned values here when done)
#if OUTER_JUNCTION_ONLY
#define KP_DEFAULT         105.0f   // PD over 3 sensors
#define HARD_ERROR         2.0f     // |error| at or beyond this: pivot in place (line only under S1 or S3)
#else
#define KP_DEFAULT         75.0f   // PD over 5 sensors steers in finer steps, so a lower Kp
#define HARD_ERROR         3.0f
#endif
#define KD_DEFAULT         1.00f     //Damper
#define CONTROL_PERIOD_MS  10U
#define PWM_MAX            700

// Dashed line gap + 90 degree corner
#define GAP_CROSS_MS       500U   //not called  // straight-ahead hold when the line was centred (dashed gap)
#define CORNER_FWD_MS      90U     // straight-ahead time before pivoting at a corner (~11 cm at 20 cm/s) (tune: KC<ms>;)
#define PIVOT_TIMEOUT_MS   1500U    // give up if the pivot finds no line
#define RECOVERY_MIN_SPEED 310      // slowest pivot speed
#define CORNER_PIVOT_SPEED 300

// Side bars (branches / corners)
#define BAR_LOCKOUT_MS     250U     // ignore a new bar event for 250ms after the last one
#define MASK_EXTRA_MS      60U      // keep ignoring the bar-side sensor this long after the bar ends

// Ambiguous patterns (e.g. while sweeping across lines during a sharp turn)
#define AMBIG_HOLD_MS      100U     // keep the last motor command this long before stopping

// Side branch (T-junction) turn, used by mission.c
#define BRANCH_FWD_MS      320U     // straight-ahead time from seeing the branch to starting the pivot (~13 cm at 20 cm/s) (tune: KJ<ms>;)

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} IrPin_t;

static const IrPin_t  irPin[IR_COUNT]    = { {GPIOB, GPIO_PIN_1}, {GPIOB, GPIO_PIN_0}, {GPIOB, GPIO_PIN_13},
                                             {GPIOB, GPIO_PIN_14}, {GPIOB, GPIO_PIN_15} };
static const int8_t   irWeight[IR_COUNT] = { -4, -2, 0, 2, 4 };

static float kp = KP_DEFAULT;
static float kd = KD_DEFAULT;
static uint32_t cornerFwdMs = CORNER_FWD_MS; //KC
static uint32_t branchFwdMs = BRANCH_FWD_MS; //KJ
uint32_t cornerSpeed = CORNER_PIVOT_SPEED;

static int lastDir = 0;   /* -1 = left; 1 = right; 0 = centred / unknown */ //Remembers the most recently selected steering direction
static uint8_t holdOk = 0;      //Not called  /* 1 = line was centred or on a gentle edge, keep going straight first */
static uint32_t lostStart = 0; //Timestamp when the current line-loss period began
static uint8_t recovering = 0; //Indicates the robot is in a line-loss recovery period

// Ambiguous pattern timer
static uint8_t  ambiguous = 0; //Indicates an ambiguous sensor pattern is currently being tracked
static uint32_t ambigStart = 0; //Timestamp when the ambiguous pattern started

// Corner counting
static uint8_t cornerCount = 0;        /* Number of completed corner pivots	 */
static uint8_t cornerPivoting = 0;     /* 1 = currently pivoting at a corner */

// Side bar tracking
static int8_t  pendingBar = 0;         /* event waiting for mission.c: -1 left, +1 right */
static int8_t  prevBarNow = 0; //Remembers the bar classification from the previous controller update ie. corner or branch
static int8_t  barLatch = 0;           /* side of the bar we are crossing / just crossed (kept through an all-white corner) */
static uint32_t lastBarEvent = 0; //Timestamp of the last accepted bar event
static uint32_t lastBarOn = 0; //Timestamp when the bar was last detected

// Controller memory
static float previousError = 0; //Stores the previous PD line-position error
static uint8_t previousValid = 0; //Indicates whether previousError is valid to be used for derivative calculation
static uint32_t lastControlMs = 0; //Time of the most recent controller update

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

//Rear sensor (1 = sees the line)
uint8_t lineReadRear(void)
{
    return irRead(REAR_IR_PORT, REAR_IR_PIN);
}

//Front centre sensor AND rear sensor both on the line = the robot is lined up with it
uint8_t lineAligned(void)
{
    return (irRead(irPin[2].port, irPin[2].pin) && irRead(REAR_IR_PORT, REAR_IR_PIN)) ? 1U : 0U;
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

//Send sensor status through bluetooth (only when IR_DEBUG is 1)
static void btReportIR(uint8_t bits, const char *action, int leftSpeed, int rightSpeed)
{
#if IR_DEBUG
    static uint32_t lastReport = 0;
    uint32_t now = HAL_GetTick();
    if (now - lastReport >= IR_REPORT_MS)
    {
        char pat[IR_COUNT + 1];
        char line[120]; // Increased size to fit the speeds
        for (uint8_t i = 0; i < IR_COUNT; i++)
            pat[i] = ((bits >> i) & 1U) ? '1' : '0';
        pat[IR_COUNT] = '\0';

        // Added L and R speed to the print
        snprintf(line, sizeof(line), "[IR] %s | %s | L:%d R:%d\r\n", pat, action, leftSpeed, rightSpeed);
        btPrint(line);
        lastReport = now;
    }
#else
    (void)bits; (void)action; (void)leftSpeed; (void)rightSpeed;
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
	const uint8_t leftBar  = (s[0] && (s[1] || s[2])) ? 1U : 0U;
	const uint8_t rightBar = (s[4] && (s[3] || s[2])) ? 1U : 0U;
#else
	const uint8_t leftBar  = (s[0] && (s[1] && s[2])) ? 1U : 0U;
	const uint8_t rightBar = (s[4] && (s[3] && s[2])) ? 1U : 0U;
#endif
	int8_t barNow = 0;
	if (leftBar && rightBar) barNow = 2;
	else if (leftBar)        barNow = -1;
	else if (rightBar)       barNow = 1;

	if (barNow == -1 || barNow == 1)
	{
		if (prevBarNow == 0 && (now - lastBarEvent) >= BAR_LOCKOUT_MS)
		{
			pendingBar = barNow;
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
	s[0] = 0;
	s[4] = 0;
#endif

	/* ---- Ignore the steering sensor on the bar side while the bar passes ---- */
	uint8_t maskActive = 0;
	if (barLatch != 0 && (barNow == barLatch || (now - lastBarOn) < MASK_EXTRA_MS))
		maskActive = 1;

	if (maskActive)
	{
#if OUTER_JUNCTION_ONLY
		if (barLatch < 0) { s[1] = 0; }
		else              { s[3] = 0; }
#else
		if (barLatch < 0) { s[0] = 0; s[1] = 0; }
		else              { s[3] = 0; s[4] = 0; }
#endif
	}

	uint8_t count = 0;
	for (uint8_t i = 0; i < IR_COUNT; i++)
		count += s[i];

	if (barLatch != 0 && !maskActive && !cornerPivoting && count > 0)
		barLatch = 0;

	if (barLatch != 0)
	{
		lastDir = barLatch;
		holdOk = 1;
	}

	const uint8_t amb = (count > 0) ? isAmbiguous(s) : 0U;
	if (!amb) ambiguous = 0;

	const char *action = "PD";
	int actLeft = 0;   // <--- Track actual Left PWM
	int actRight = 0;  // <--- Track actual Right PWM

	if (count > 0) //Not all white
	{
		if (cornerPivoting && s[2])
		{
			cornerCount++;
			cornerPivoting = 0;
			barLatch = 0;
			recovering = 0;
		}
		else if (!cornerPivoting && count > 0)
		{
	    	    recovering = 0;
		}
	}

	// Wide bar = both outer sensors on (crossing or end bar)
	if ((raw & 0x11U) == 0x11U)
	{
		forward(speed);
		actLeft = speed; actRight = speed; // Track
		previousValid = 0;
		lastDir = 0;
		holdOk = 1;
		action = "CROSSING";
	}

	// All white (00000): either right after a corner bar, or a dotted-line gap / brief dropout
	else if (count == 0)
	{
		previousValid = 0;

		if (!recovering)
		{
			lostStart = now;
			recovering = 1;
		}

		const uint8_t afterBar = (barLatch != 0) ? 1U : 0U;
		const uint32_t lostMs = now - lostStart;

		if (afterBar || cornerCount > 0)
		{
			action = "CORNER - SEEN";
			if (!cornerPivoting && lostMs < cornerFwdMs)
			{
				forward(speed);
				actLeft = speed; actRight = speed; // Track
				action = "CORNER - FORWARD";
			}
			else if ((lostMs - cornerFwdMs) >= PIVOT_TIMEOUT_MS)
			{
				stopMotors();
				actLeft = 0; actRight = 0; // Track
				barLatch = 0;
				cornerPivoting = 0;
				action = "CORNER - TIMEOUT";
			}
			else
			{
				cornerPivoting = 1;
				if (lastDir == -1)
				{
					turnLeft(CORNER_PIVOT_SPEED);
					actLeft = -CORNER_PIVOT_SPEED; actRight = CORNER_PIVOT_SPEED; // Track
					action = "CORNER - PIVOT LEFT";
				}
				else
				{
					turnRight(CORNER_PIVOT_SPEED);
					actLeft = CORNER_PIVOT_SPEED; actRight = -CORNER_PIVOT_SPEED; // Track
					action = "CORNER - PIVOT RIGHT";
				}
			}
		}
		else
		{
			if (lostMs >= PIVOT_TIMEOUT_MS)
			{
				stopMotors();
				actLeft = 0; actRight = 0; // Track
				action = "LOST - TIMEOUT";
			}
			else
			{
				int slowSpeed = speed / 2;
				if (slowSpeed < RECOVERY_MIN_SPEED)
					slowSpeed = RECOVERY_MIN_SPEED;

				forward(slowSpeed);
				actLeft = slowSpeed; actRight = slowSpeed; // Track
				action = "GAP / CREEP FORWARD";
			}
		}
	}

	else if (cornerPivoting)
	{
		if ((now - lostStart) >= (cornerFwdMs + PIVOT_TIMEOUT_MS))
		{
			stopMotors();
			actLeft = 0; actRight = 0; // Track
			barLatch = 0;
			cornerPivoting = 0;
			action = "CORNER - TIMEOUT";
		}
		else
		{
			if (lastDir == -1)
			{
				turnLeft(CORNER_PIVOT_SPEED);
				actLeft = -CORNER_PIVOT_SPEED; actRight = CORNER_PIVOT_SPEED; // Track
			}
			else
			{
				turnRight(CORNER_PIVOT_SPEED);
				actLeft = CORNER_PIVOT_SPEED; actRight = -CORNER_PIVOT_SPEED; // Track
			}
			action = "CORNER - PIVOT (LINE SEEN)";
		}
	}
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
			actLeft = 0; actRight = 0; // Track
			action = "AMBIGUOUS - STOP";
		}
		else
		{
			action = "AMBIGUOUS - HOLD";
		}
	}
	else
	{
		float sum = 0.0f;
		for (uint8_t i = 0; i < IR_COUNT; i++)
			sum += (float)irWeight[i] * (float)s[i];

		const float error = sum / (float)count;

		if (error <= -HARD_ERROR)
		{
			turnLeft(speed);
			actLeft = -speed; actRight = speed; // Track
			lastDir = -1;
			holdOk = 0;
			previousValid = 0;
			action = "HARD LEFT";
		}
		else if (error >= HARD_ERROR)
		{
			turnRight(speed);
			actLeft = speed; actRight = -speed; // Track
			lastDir = 1;
			holdOk = 0;
			previousValid = 0;
			action = "HARD RIGHT";
		}
		else
		{
			float dt = elapsed / 1000.0f;
			float derivative = 0.0f;

			if (previousValid)
			{
				derivative = (error - previousError) / dt;
			}

			float correction = kp * error + kd * derivative;

			int leftSpeed = clampSpeed((int)(speed + correction));
			int rightSpeed = clampSpeed((int)(speed - correction));

			setLeft(leftSpeed);
			setRight(rightSpeed);
			actLeft = leftSpeed; actRight = rightSpeed; // Track

			previousError = error;
			previousValid = 1;
			holdOk = 1;

			if (barLatch != 0)
			{
				lastDir = barLatch;
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

	// Pass the speeds to the debug function
	btReportIR(raw, action, actLeft, actRight);
}

/* ---------------------------------------------------------------------------
 * Live tuning over Bluetooth (end each entry with ; or Enter)
 *
 *   T            shows all tuned values immediately
 *   KP350;       Kp = 350
 *   KD0.35;      Kd = 0.35
 *   KC550;       corner straight-ahead time, ms
 *   KJ650;       branch straight-ahead time, ms
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
             "[TUNE] Kp=%d Kd=%d.%02d Corner=%lu Branch=%lu ms CornerSpeed=%i\r\n",
             (int)(kp + 0.5f), kdHundredths / 100, kdHundredths % 100,
             (unsigned long)cornerFwdMs, (unsigned long)branchFwdMs), (int)cornerSpeed;
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
        return 0;
    }

    /* stop commands always win, even mid-entry */
    if (c == 'X' || c == 'x' || c == ' ')
    {
        entryState = 0;
        return 0;
    }

    entryTick = HAL_GetTick();

    if (entryState == 1)                       /* after K: expect P, D, C, J */
    {
        if (isEnd) { entryState = 0; printGains(); return 1; }

        char up = c;
        if (up >= 'a' && up <= 'z') up = (char)(up - 32);

        if (up == 'P' || up == 'D' || up == 'C' || up == 'J' || up == 'A')
        {
            entryTarget = up;
        }
        else
        {
            entryState = 0;
            btPrint("[TUNE] KP KD KC KJ then a number and ;\r\n");
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
        else if (entryTarget == 'A') cornerSpeed = (uint32_t)((v > 700) ? MS_MAX : v);
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
