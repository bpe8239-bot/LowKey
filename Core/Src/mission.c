/*
 * mission.c  -  mission script runner
 * ...
 */

#include "mission.h"
#include "main.h"
#include "line_following.h"
#include "colour.h"
#include "motor.h"
#include "bluetooth.h"
#include <stdio.h>

#define BRANCH_COUNT_TIMEOUT_MS  20000U
#define BRANCH_PIVOT_BLIND_MS    300U
#define BRANCH_PIVOT_TIMEOUT_MS  3000U  /* Increased to 3000U to give it more time to turn */
#define END_BAR_TIMEOUT_MS       8000U
#define END_BAR_CONFIRM_MS       30U
#define PIVOT_MIN_SPEED          400
#define CENTRE_BIT               0x04U
#define END_BAR_BITS             0x15U
#define BRANCH_SPEED             390
#define BRANCH_MIN_SPEED         380
#define BRANCH_END_TIMEOUT_MS    4000U
#define CORNER_SETTLE_MS         250U

#define T1_DEFAULT_MS            400U
#define TS_DEFAULT_MS            1000U
#define WIN_DEFAULT_MS           350U

typedef enum
{
    ST_FOLLOW_CORNERS = 0,
    ST_BRANCH,
    ST_END_BAR,
    ST_STOP_CORNER,
    ST_FINISH
} StepType_t;

typedef struct
{
    StepType_t type;
    int8_t     side;
    uint8_t    n;
} Step_t;

static const Step_t basicScript[] =
{
    { ST_FOLLOW_CORNERS,  0, 2 },
    { ST_BRANCH,         -1, 0 },
    { ST_STOP_CORNER,     0, 0 },
    { ST_FINISH,          0, 0 }
};

static const Step_t *script = basicScript;
static uint8_t scriptLen = (uint8_t)(sizeof(basicScript) / sizeof(basicScript[0]));

typedef enum { BR_COUNT = 0, BR_FWD, BR_PIVOT } BranchState_t;

static uint8_t       active = 0;
static uint8_t       stepIdx = 0;
static uint32_t      stateStart = 0;

static uint8_t       target = 0;

static uint8_t       cornerBase = 0;
uint8_t              cornersSeen = 0;
static BranchState_t brState = BR_COUNT;
static uint8_t       branchCount = 0;
static uint8_t       sawWhite = 0;
static uint8_t       barSeen = 0;
static uint32_t      barStart = 0;
static uint8_t       arrivedAligned = 0;

static uint32_t t1Ms = T1_DEFAULT_MS;
static uint32_t tsMs = TS_DEFAULT_MS;
static uint32_t winMs = WIN_DEFAULT_MS;
static uint32_t nextExpected = 0;
static uint32_t lastEventTick = 0;
static uint8_t  lastWasReal = 0;
static uint32_t tsEst = TS_DEFAULT_MS;

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
            cornersSeen = 0;
            break;

        case ST_BRANCH:
            branchCount = 0;
            brState = BR_COUNT;
            (void)lineBarEvent();
            nextExpected = now + t1Ms;
            lastEventTick = 0;
            lastWasReal = 0;
            tsEst = tsMs;
            break;

        case ST_STOP_CORNER:
            (void)lineBarEvent();
            btPrint("[MISSION] Entered ST_STOP_CORNER (Blind window started)\r\n");
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
    lineLogClear();
    lineFollowReset();
    active = 1;
    stepIdx = 0;
    target = 0;
    startStep();
    btPrint("[MISSION] START - reading colour card\r\n");
}

static int branchSpeed(int speed)
{
    int s = speed / 2;
    if (s < BRANCH_MIN_SPEED) s = BRANCH_MIN_SPEED;
    if (s > BRANCH_SPEED)     s = BRANCH_SPEED;
    return s;
}

uint8_t missionTick(int speed)
{
    if (!active) return 0;

    const uint32_t now = HAL_GetTick();
    const uint8_t bits = lineReadBits();
    const Step_t *s = &script[stepIdx];
    char line[100];

    readColour();

    switch (s->type)
    {
    case ST_FOLLOW_CORNERS:
    {
        (void)lineBarEvent();
        lineFollow(speed);
        const uint8_t turned = lineFollowCorners() - cornerBase;

        if (turned > cornersSeen)
        {
            cornersSeen = turned;
            stateStart = now;
            char msg[60];
            snprintf(msg, sizeof(msg), "Corner Seen %u\r\n", (unsigned)cornersSeen);
            btPrint(msg);
        }

        if (target == 0 && turned >= 1)
            return fail("[MISSION] FAILED - no colour read before the first corner\r\n");

        if (turned >= s->n)
        {
            if ((now - stateStart) >= CORNER_SETTLE_MS)
                nextStep();
        }
        return 1;
    }

    case ST_BRANCH:
    {
        if (target == 0)
            return fail("[MISSION] FAILED - no target\r\n");

        switch (brState)
        {
            case BR_COUNT:
            {
                const int8_t ev = lineBarEvent();
                lineFollow(branchSpeed(speed));

                if (ev == s->side)
                {
                    if ((now + winMs) < nextExpected)
                    {
                        btPrint("[MISSION] Bar ignored (too early)\r\n");
                    }
                    else
                    {
                        if (lastWasReal && lastEventTick != 0U)
                        {
                            uint32_t measured = now - lastEventTick;
                            if (measured < tsMs / 2U) measured = tsMs / 2U;
                            if (measured > tsMs * 2U) measured = tsMs * 2U;
                            tsEst = measured;
                        }
                        branchCount++;
                        snprintf(line, sizeof(line), "[MISSION] Branch %u (target %u)\r\n",
                                 (unsigned)branchCount, (unsigned)target);
                        btPrint(line);

                        lastEventTick = now;
                        lastWasReal = 1;
                        nextExpected = now + tsEst;

                        if (branchCount == target)
                        {
                            brState = BR_FWD;
                            stateStart = now;
                            btPrint("[MISSION] Target found! Entering BR_FWD.\r\n");
                        }
                    }
                }

                if ((now - stateStart) >= BRANCH_COUNT_TIMEOUT_MS && brState == BR_COUNT)
                    return fail("[MISSION] FAILED - target branch not found\r\n");
                return 1;
            }

            case BR_FWD:
            {
                (void)lineBarEvent();
                lineFollow(branchSpeed(speed));
                uint32_t fwdElapsed = now - stateStart;
                uint32_t fwdTarget = lineFollowBranchFwdMs();

                if (fwdElapsed >= fwdTarget)
                {
                    brState = BR_PIVOT;
                    stateStart = now;
                    sawWhite = 0;

                    btPrint("[MISSION] FWD complete. Turning into branch...\r\n");
                    if (s->side < 0) btPrint("[MISSION] Action: turnLeft()\r\n");
                    else             btPrint("[MISSION] Action: turnRight()\r\n");
                }
                return 1;
            }

            case BR_PIVOT:
            default:
            {
                int pivotSpeed = 480;
                if (pivotSpeed < PIVOT_MIN_SPEED) pivotSpeed = PIVOT_MIN_SPEED;

                if (s->side < 0) turnLeft(pivotSpeed);
                else             turnRight(pivotSpeed);

                // Debug: Log when we finally see white
                if (!(bits & CENTRE_BIT) && !sawWhite) {
                    sawWhite = 1;
                    snprintf(line, sizeof(line), "[PIVOT] Saw WHITE gap! (bits=%d)\r\n", bits);
                    btPrint(line);
                }

                // Debug: Log the exit condition
                if ((now - stateStart) >= BRANCH_PIVOT_BLIND_MS && sawWhite && (bits & CENTRE_BIT))
                {
                    snprintf(line, sizeof(line), "[PIVOT] Found branch line! (bits=%d). Exiting pivot.\r\n", bits);
                    btPrint(line);
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

    case ST_STOP_CORNER:
    {
        const int8_t ev = lineBarEvent();
        const uint32_t cornerElapsed = now - stateStart;

        if (cornerElapsed > 1500U)
        {
            if (ev != 0 || (bits & END_BAR_BITS) == END_BAR_BITS)
            {
                snprintf(line, sizeof(line), "[STOP] Triggered! ev=%d, bits=%d (0x%02X)\r\n", ev, bits, bits);
                btPrint(line);
                arrivedAligned = lineAligned();
                stopMotors();
                nextStep();
                return 1;
            }
        }
        else
        {
            (void)lineBarEvent();
            // Just for debug: print when we are safe in the blind window
            // You can comment this out later, but leave it for the test
            if (cornerElapsed < 200U) {
                 snprintf(line, sizeof(line), "[STOP] Blind window, ignoring event. bits=%d\r\n", bits);
                 btPrint(line);
            }
        }

        if ((now - stateStart) >= END_BAR_TIMEOUT_MS)
            return fail("[MISSION] FAILED - corner after branch not found\r\n");

        lineFollow(speed);
        return 1;
    }

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

    case ST_FINISH:
    default:
        return finish();
    }
}
