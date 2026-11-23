/*
 * oled.c
 *
 *  Created on: 20 Nov 2026
 *      Author: bpe82
 */
#include "oled.h"
#include "main.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c2;

#define OLED_I2C_ADDR       (0x3CU << 1)
#define OLED_MIN_FRAME_MS   350U
#define OLED_POPUP_MS       1500U
#define OLED_EVENT_MS       1800U
#define OLED_COLOUR_POPUP_MS 2000U

static uint8_t ready = 0U;
static uint8_t dirty = 1U;
static uint32_t lastFrame = 0U;
static int lastPwm = -1;
static char modeText[22] = "MANUAL";
static char statusText[22] = "READY";
static char eventText[22] = "";
static uint8_t eventActive = 0U;
static uint32_t eventStart = 0U;
static char popupTitle[22] = "";
static char popupValue[22] = "";
static uint8_t popupActive = 0U;
static uint32_t popupStart = 0U;
static uint32_t popupDurationMs = OLED_POPUP_MS;

static void copyText(char *dest, size_t capacity, const char *src)
{
    if (src == NULL) src = "";
    snprintf(dest, capacity, "%.21s", src);
}

static void writeLine(uint8_t y, const char *src)
{
    /* Font_6x8 fits up to 21 characters across a 128-pixel screen. */
    char clipped[22];
    copyText(clipped, sizeof(clipped), src);
    ssd1306_SetCursor(0, y);
    ssd1306_WriteString(clipped, Font_6x8, White);
}

void OLED_UI_Init(void)
{
    /* Avoid entering the library if the address doesn't acknowledge. */
    if (HAL_I2C_IsDeviceReady(&hi2c2, OLED_I2C_ADDR, 2, 25) != HAL_OK)
    {
        ready = 0U;
        return;
    }
    ssd1306_Init();
    ready = 1U;
    dirty = 1U;
    lastFrame = HAL_GetTick() - OLED_MIN_FRAME_MS;
}

void OLED_UI_SetStatus(const char *status)
{
    char next[22];
    copyText(next, sizeof(next), status);
    if (strcmp(next, statusText) != 0)
    {
        strcpy(statusText, next);
        dirty = 1U;
    }
}

void OLED_UI_EventStatus(const char *event)
{
    copyText(eventText, sizeof(eventText), event);
    eventStart = HAL_GetTick();
    eventActive = 1U;
    dirty = 1U;
}

static void startPopup(const char *title, const char *value, uint32_t durationMs)
{
    copyText(popupTitle, sizeof(popupTitle), title);
    copyText(popupValue, sizeof(popupValue), value);
    popupStart = HAL_GetTick();
    popupDurationMs = durationMs;
    popupActive = 1U;
    dirty = 1U;
}

void OLED_UI_SetMode(const char *mode)
{
    char next[22];
    copyText(next, sizeof(next), mode);
    if (strcmp(next, modeText) != 0)
    {
        strcpy(modeText, next);
        startPopup("MODE SWITCHED", modeText, OLED_POPUP_MS	);
    }
}

void OLED_UI_Claw(const char *operation)
{
    startPopup("CLAW", operation, OLED_POPUP_MS);
}

/* Call once when mission logic CONFIRMS a colour, not each sensor poll. */
void OLED_UI_ColourDetected(const char *colour)
{
    startPopup("COLOUR DETECTED", colour, OLED_COLOUR_POPUP_MS);
}

void OLED_UI_Tick(int pwm) //main OLED display function
{
    if (!ready) return;

    uint32_t now = HAL_GetTick();
    if (popupActive && (uint32_t)(now - popupStart) >= popupDurationMs)
    {
        popupActive = 0U;
        dirty = 1U;
    }
    if (eventActive && (uint32_t)(now - eventStart) >= OLED_EVENT_MS)
    {
        eventActive = 0U;
        dirty = 1U;
    }
    if (pwm != lastPwm)
    {
        lastPwm = pwm;
        dirty = 1U;
    }
    if (!dirty || (uint32_t)(now - lastFrame) < OLED_MIN_FRAME_MS)
        return;

    lastFrame = now;
    dirty = 0U;
    ssd1306_Fill(Black);

    if (popupActive)
    {
        writeLine(18, popupTitle);
        writeLine(38, popupValue);
    }
    else
    {
        char line[22];

        writeLine(0, "STATUS:");
        writeLine(20, eventActive ? eventText : statusText);
        snprintf(line, sizeof(line), "SPEED: %d", pwm);
        writeLine(48, line);
    }
    ssd1306_UpdateScreen();
}
