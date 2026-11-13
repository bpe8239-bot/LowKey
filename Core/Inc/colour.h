/*
 * colour.h
 *
 *  Created on: Nov 9, 2026
 *      Author: Zeyou Chin
 */

#ifndef INC_COLOUR_H_
#define INC_COLOUR_H_

#include "main.h"
#include <stdio.h>

#define TCS34725_I2C_ADDR       (0x29 << 1)

typedef enum {
    COLOUR_UNKNOWN = 0,
    COLOUR_RED,
    COLOUR_GREEN,
    COLOUR_BLUE
} DetectedColour_t;

// RGBC Raw Data Structure
typedef struct {
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;
} ColourData_t;

extern float tuning_red_margin;
extern float tuning_red_min;
extern float tuning_green_min;
extern float tuning_blue_min;
extern float tuning_blue_max;
extern float tuning_rg_diff;

HAL_StatusTypeDef ColourSensor_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ColourSensor_ReadRaw(ColourData_t *data);
DetectedColour_t  ColourSensor_GetColour(void);
void ColourSensor_CalibrateWhite(void);

#endif /* INC_COLOUR_H_ */
