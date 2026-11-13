/*
 * mission.c  -  mission script runner
 *
 * The mission is a list of STEPS. Each step is one reusable move. The basic map is just
 * one short script; the advanced map will be a longer script built from the same steps
 * (plus a few new ones: claw, U-turn, ...).
 *
 *   ST_FOLLOW_CORNERS  follow the line until n corners have been turned
 *   ST_BRANCH          count side branches on one side (-1 left, +1 right) up to the target
 *                      number, turn into that branch, stop when the line is found
 *   ST_END_BAR         follow until the wide bar (S0, S2 and S4 on), then stop
 *   ST_FINISH          mission over, hand control back to manual
 *
 * Basic map (Figure 1):
 *   start -> east (dashed section, colour card read on the way) -> top-right corner (right)
 *   -> bottom-right corner (right) -> west along the bottom, branches on the LEFT
 *   (first branch = ball 1, second = ball 2, third = ball 3) -> end bar -> manual.
 *   Colour: RED = ball 1, BLUE = ball 2, GREEN = ball 3.
 *
 * Branch counting:
 *   PRIMARY  the pattern bar events from line_following.c (111xx on the left).
 *   BACKUP   a time window. Each branch is expected at a known time after the previous one:
 *              - an event far too early is ignored (not a real branch)
 *              - an event inside the window is counted, and the clock re-syncs to it
 *              - no event by the end of the window: the branch is counted as missed
 *            If the missed branch is the target, the robot still turns at the predicted time.
 *
 * Adding the advanced map later = write another script table and add the new step types
 * (ST_CLAW_CLOSE, ST_CLAW_OPEN, ST_UTURN, ...) to the switch in missionTick().
 */

#include "mission.h"
#include "main.h"
#include "line_following.h"
#include "colour.h"
#include "motor.h"
#include "bluetooth.h"
#include <stdio.h>

#define BRANCH_COUNT_TIMEOUT_MS  20000U /* give up if the branch step takes this long */
#define BRANCH_PIVOT_BLIND_MS    150U   /* during the branch pivot, ignore the centre sensor for this long */
#define BRANCH_PIVOT_TIMEOUT_MS  2000U  /* give up if the pivot never finds the branch */
#define END_BAR_TIMEOUT_MS       8000U  /* give up if the end bar is never found */
#define END_BAR_CONFIRM_MS       30U    /* S0, S2 and S4 on for this long = end bar */
#define PIVOT_MIN_SPEED          300
#define CENTRE_BIT               0x04U  /* bit2 = centre sensor */
#define END_BAR_BITS             0x15U  /* bit0 + bit2 + bit4 */

/* Time-based backup, defaults for about 20 cm/s. Measure with the L key and tune with KA / KB / KW. */
#define T1_DEFAULT_MS            400U   /* end of 2nd corner -> first branch bar */
#define TS_DEFAULT_MS            1000U  /* branch to branch (20 cm) */
#define WIN_DEFAULT_MS           350U   /* +/- window */

typedef enum
{
    ST_FOLLOW_CORNERS = 0,
    ST_BRANCH,
    ST_END_BAR,
    ST_FINISH
} StepType_t;

typedef struct
{
    StepType_t type;
    int8_t     side;    /* ST_BRANCH: -1 = left, +1 = right */
    uint8_t    n;       /* ST_FOLLOW_CORNERS: number of corners */
} Step_t;

/* ---- Basic map script ---- */
static const Step_t basicScript[] =
{
    { ST_FOLLOW_CORNERS,  0, 2 },   /* top-right and bottom-right corners; colour read on the way */
    { ST_BRANCH,         -1, 0 },   /* branches on the left; count up to the ball number */
    { ST_END_BAR,         0, 0 },   /* follow the branch to the end bar */
    { ST_FINISH,          0, 0 }
};

static const Step_t *script = basicScript;
static uint8_t scriptLen = (uint8_t)(sizeof(basicScript) / sizeof(basicScript[0]));

typedef enum { BR_COUNT = 0, BR_FWD, BR_PIVOT } BranchState_t;

static uint8_t       active = 0;
static uint8_t       stepIdx = 0;
static uint32_t      stateStart = 0;

static uint8_t       target = 0;         /* ball number 1..3, 0 = colour not read yet */

/* per-step working variables */
static uint8_t       cornerBase = 0;
static BranchState_t brState = BR_COUNT;
static uint8_t       branchCount = 0;
static uint8_t       sawWhite = 0;
static uint8_t       barSeen = 0;
static uint32_t      barStart = 0;
static uint8_t 		 arrivedAligned = 0;

/* time-based backup */
static uint32_t t1Ms = T1_DEFAULT_MS;
static uint32_t tsMs = TS_DEFAULT_MS;
static uint32_t winMs = WIN_DEFAULT_MS;
static uint32_t nextExpected = 0;        /* tick when the next branch bar is expected */
static uint32_t lastEventTick = 0;       /* tick of the last real (detected) branch */
static uint8_t  lastWasReal = 0;
static uint32_t tsEst = TS_DEFAULT_MS;   /* branch interval, re-measured during the run */

void missionSetParam(char id, uint32_t value)
{
    switch (id)
    {
        case 'A': t1Ms = value;  break;
        case 'B': tsMs = value;  break;
        case 'W': winMs = value; break;
        default: break;
    }
}

uint32_t missionGetParam(char id)
{
    switch (id)
    {
        case 'A': return t1Ms;
        case 'B': return tsMs;
        case 'W': return winMs;
        default:  return 0;
    }
}

static uint8_t finish(void)
{
	 char line[96];
	    stopMotors();
	    snprintf(line, sizeof(line), "[MISSION] ARRIVED at ball %u (%s) - MANUAL MODE\r\n",
	             (unsigned)target, arrivedAligned ? "aligned" : "NOT aligned");
	    btPrint(line);
    active = 0;
    return 0;
}

static uint8_t fail(const char *msg)
{
    stopMotors();
    btPrint(msg);
    active = 0;
    return 0;
}

static void startStep(void)
{
    const uint32_t now = HAL_GetTick();
    stateStart = now;

    switch (script[stepIdx].type)
    {
        case ST_FOLLOW_CORNERS:
            cornerBase = lineFollowCorners();
            break;

        case ST_BRANCH:
            branchCount = 0;
            brState = BR_COUNT;
            (void)lineBarEvent();          /* drop any bar event left over from the previous step */
            nextExpected = now + t1Ms;
            lastEventTick = 0;
            lastWasReal = 0;
            tsEst = tsMs;
            break;

        case ST_END_BAR:
            barSeen = 0;
            break;

        default:
            break;
    }
}

static void nextStep(void)
{
    if (stepIdx + 1U < scriptLen)
        stepIdx++;
    startStep();
}

/* Reads the colour card once: the first confirmed colour wins */
static void readColour(void)
{
    char line[64];

    if (target != 0) return;

    DetectedColour_t col = ColourSensor_GetColour();
    if (col == COLOUR_UNKNOWN) return;

    target = (col == COLOUR_RED) ? 1U : (col == COLOUR_BLUE) ? 2U : 3U;
    const char *name =
        (col == COLOUR_RED)   ? "RED" :
        (col == COLOUR_BLUE)  ? "BLUE" :
        (col == COLOUR_GREEN) ? "GREEN" :
                                "UNKNOWN";

    snprintf(line, sizeof(line),
             "[MISSION] Colour %s -> ball %u\r\n",
             name, (unsigned)target);
    btPrint(line);
}

void missionStart(void)
{
    lineLogClear();                        /* so the sensor log starts with this run */
    lineFollowReset();
    active = 1;
    stepIdx = 0;
    target = 0;
    startStep();
    btPrint("[MISSION] START - reading colour card\r\n");
}

uint8_t missionTick(int speed)
{
    if (!active) return 0;

    const uint32_t now = HAL_GetTick();
    const uint8_t bits = lineReadBits();
    const Step_t *s = &script[stepIdx];
    char line[64];

    readColour();

    switch (s->type)
    {
        /* ------------------------------------------------------------------ */
        case ST_FOLLOW_CORNERS:
        {
            (void)lineBarEvent();                       /* bar events don't matter here */
            lineFollow(speed);

            const uint8_t turned = (uint8_t)(lineFollowCorners() - cornerBase);

            if (target == 0 && turned >= 1)
                return fail("[MISSION] FAILED - no colour read before the first corner\r\n");

            if (turned >= s->n)
                nextStep();
            return 1;
        }

        /* ------------------------------------------------------------------ */
        case ST_BRANCH:
        {
            if (target == 0)
                return fail("[MISSION] FAILED - no target\r\n");

            switch (brState)
            {
                case BR_COUNT:
                {
                    const int8_t ev = lineBarEvent();
                    lineFollow(speed);                  /* masks the bar side so we drive straight past it */

                    if (ev == s->side)
                    {
                        if ((now + winMs) < nextExpected)
                        {
                            /* far too early for the next branch: not a real one (also removes duplicates) */
                            btPrint("[MISSION] Bar ignored (too early)\r\n");
                        }
                        else
                        {
                            /* a real branch: re-measure the interval and re-sync the clock */
                            if (lastWasReal && lastEventTick != 0U)
                            {
                                uint32_t measured = now - lastEventTick;
                                if (measured < tsMs / 2U) measured = tsMs / 2U;
                                if (measured > tsMs * 2U) measured = tsMs * 2U;
                                tsEst = measured;
                            }
                            branchCount++;
                            lastEventTick = now;
                            lastWasReal = 1;
                            nextExpected = now + tsEst;

                            snprintf(line, sizeof(line), "[MISSION] Branch %u (target %u)\r\n",
                                     (unsigned)branchCount, (unsigned)target);
                            btPrint(line);

                            if (branchCount == target)
                            {
                                brState = BR_FWD;
                                stateStart = now;
                            }
                        }
                    }
                    else if (now > (nextExpected + winMs))
                    {
                        /* the expected branch never showed up: count it as missed */
                        branchCount++;
                        lastWasReal = 0;

                        snprintf(line, sizeof(line), "[MISSION] Branch %u assumed (missed)\r\n",
                                 (unsigned)branchCount);
                        btPrint(line);

                        if (branchCount == target)
                        {
                            brState = BR_FWD;
                            stateStart = nextExpected;  /* as if it had been seen at the expected time */
                        }
                        nextExpected += tsEst;
                    }

                    if ((now - stateStart) >= BRANCH_COUNT_TIMEOUT_MS && brState == BR_COUNT)
                        return fail("[MISSION] FAILED - target branch not found\r\n");
                    return 1;
                }

                case BR_FWD:
                    (void)lineBarEvent();
                    lineFollow(speed);                  /* keep going straight so the wheels reach the branch */
                    if ((now - stateStart) >= lineFollowBranchFwdMs())
                    {
                        brState = BR_PIVOT;
                        stateStart = now;
                        sawWhite = 0;
                        btPrint("[MISSION] Turning into branch\r\n");
                    }
                    return 1;

                case BR_PIVOT:
                default:
                {
                    int pivotSpeed = speed / 2;
                    if (pivotSpeed < PIVOT_MIN_SPEED) pivotSpeed = PIVOT_MIN_SPEED;

                    if (s->side < 0) turnLeft(pivotSpeed);
                    else             turnRight(pivotSpeed);

                    if (bits == 0) sawWhite = 1;

                    /* done when the centre sensor finds the branch line, after leaving the main line */
                    if (sawWhite && (bits & CENTRE_BIT) && (now - stateStart) >= BRANCH_PIVOT_BLIND_MS)
                    {
                        lineFollowReset();
                        nextStep();
                        return 1;
                    }
                    if ((now - stateStart) >= BRANCH_PIVOT_TIMEOUT_MS)
                        return fail("[MISSION] FAILED - branch line not found after turning\r\n");
                    return 1;
                }
            }
        }

        /* ------------------------------------------------------------------ */
        case ST_END_BAR:
        {
            if ((bits & END_BAR_BITS) == END_BAR_BITS)
            {
                if (!barSeen) { barSeen = 1; barStart = now; }
                if ((now - barStart) >= END_BAR_CONFIRM_MS)
                {
                	arrivedAligned = lineAligned();
                	stopMotors();
                    nextStep();
                    return 1;
                }
            }
            else
            {
                barSeen = 0;
            }

            if ((now - stateStart) >= END_BAR_TIMEOUT_MS)
                return fail("[MISSION] FAILED - end bar not found\r\n");

            (void)lineBarEvent();
            lineFollow(speed);
            return 1;
        }

        /* ------------------------------------------------------------------ */
        case ST_FINISH:
        default:
            return finish();
    }
}
