/*
 * colour.h
 *
 *  Created on: Nov 9, 2026
 *      Author: AutoCar System
 */

#ifndef INC_COLOUR_H_
#define INC_COLOUR_H_

#include "main.h"

// TCS34725 I2C Address (7-bit address 0x29 shifted left by 1)
#define TCS34725_I2C_ADDR       (0x29 << 1)

// Enumerated Return Types for Parent Functions
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

/* Function Prototypes */
HAL_StatusTypeDef ColourSensor_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ColourSensor_ReadRaw(ColourData_t *data);
DetectedColour_t  ColourSensor_GetColour(void);

#endif /* INC_COLOUR_H_ */
