/*
 * colour.c
 *
 *  Created on: Nov 9, 2026
 *      Author: Zeyou Chin
 */

//PB6 - SCL
//PB7 - SDA

#include "colour.h"
#include "bluetooth.h"
#include "stdio.h"
#include <math.h>

#define TCS34725_COMMAND_BIT    0x80
#define TCS34725_ENABLE         0x00
#define TCS34725_ATIME          0x01
#define TCS34725_CONTROL        0x0F
#define TCS34725_ID             0x12
#define TCS34725_CDATAL         0x14
#define TCS34725_RDATAL         0x16
#define TCS34725_GDATAL         0x18
#define TCS34725_BDATAL         0x1A

/* Register Configurations */
#define TCS34725_ENABLE_PON     0x01  /* Power ON */
#define TCS34725_ENABLE_AEN     0x02  /* RGBC Enable */
#define TCS34725_ATIME_154MS    0xFF  /* Integration time: 154 ms */
#define TCS34725_GAIN_16X       0x02  /* Gain setting: 16x */

static I2C_HandleTypeDef *tcs_hi2c = NULL;

/**
 * @brief  Helper function to write a byte to a TCS34725 register
 */
static HAL_StatusTypeDef writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t txData[2] = { TCS34725_COMMAND_BIT | reg, value };
    return HAL_I2C_Master_Transmit(tcs_hi2c, TCS34725_I2C_ADDR, txData, 2, HAL_MAX_DELAY);
}

/**
 * @brief  Helper function to read a 16-bit word (Low byte + High byte)
 */
static HAL_StatusTypeDef readRegister16(uint8_t reg, uint16_t *value)
{
    uint8_t command = TCS34725_COMMAND_BIT | reg;
    uint8_t buffer[2] = {0};

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(tcs_hi2c, TCS34725_I2C_ADDR, &command, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    status = HAL_I2C_Master_Receive(tcs_hi2c, TCS34725_I2C_ADDR, buffer, 2, HAL_MAX_DELAY);
    if (status == HAL_OK)
    {
        *value = (uint16_t)(buffer[1] << 8) | buffer[0];
    }
    return status;
}

/**
 * @brief  Initializes the TCS34725 sensor over I2C
 */
HAL_StatusTypeDef ColourSensor_Init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) return HAL_ERROR;
    tcs_hi2c = hi2c;

    uint8_t id = 0;
    uint8_t command = TCS34725_COMMAND_BIT | TCS34725_ID;

    HAL_I2C_Master_Transmit(tcs_hi2c, TCS34725_I2C_ADDR, &command, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(tcs_hi2c, TCS34725_I2C_ADDR, &id, 1, HAL_MAX_DELAY);

    if (id != 0x44 && id != 0x4D)
    {
        return HAL_ERROR; // ID mismatch
    }

    writeRegister(TCS34725_ATIME, TCS34725_ATIME_154MS);
    writeRegister(TCS34725_CONTROL, TCS34725_GAIN_16X);

    writeRegister(TCS34725_ENABLE, TCS34725_ENABLE_PON);
    HAL_Delay(3);
    writeRegister(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);

    return HAL_OK;
}

/**
 * @brief  Reads raw RGBC 16-bit values from the sensor
 */
HAL_StatusTypeDef ColourSensor_ReadRaw(ColourData_t *data)
{
    if (tcs_hi2c == NULL || data == NULL) return HAL_ERROR;

    if (readRegister16(TCS34725_CDATAL, &data->c) != HAL_OK) return HAL_ERROR;
    if (readRegister16(TCS34725_RDATAL, &data->r) != HAL_OK) return HAL_ERROR;
    if (readRegister16(TCS34725_GDATAL, &data->g) != HAL_OK) return HAL_ERROR;
    if (readRegister16(TCS34725_BDATAL, &data->b) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

/**
 * @brief  Internal helper to evaluate a single instant reading
 */
static DetectedColour_t evaluateSingleSample(void)
{
    ColourData_t raw;
    if (ColourSensor_ReadRaw(&raw) != HAL_OK)
    {
        return COLOUR_UNKNOWN;
    }

    if (raw.c < 50)
    {
        return COLOUR_UNKNOWN;
    }

    float rRatio = (float)raw.r / (float)raw.c;
    float gRatio = (float)raw.g / (float)raw.c;
    float bRatio = (float)raw.b / (float)raw.c;

    // Red: Red dominant and well above green
    if (rRatio > (gRatio + 0.18f) && rRatio > bRatio && rRatio > 0.40f)
    {
        return COLOUR_RED;
    }
    // Green: Green dominant over red
    else if (gRatio > rRatio && gRatio > bRatio && gRatio > 0.35f)
    {
        return COLOUR_GREEN;
    }
    // Blue: Uses fabsf() for proper float comparison, and upper clear limit to ignore white floors
    else if (bRatio >= 0.20f && bRatio <= 0.35f && fabsf(rRatio - gRatio) <= 0.05f && raw.c > 100 && raw.c < 500)
    {
        return COLOUR_BLUE;
    }

    return COLOUR_UNKNOWN;
}

/**
 * @brief  Scans the colour 5 consecutive times. Returns the colour ONLY if
 *         all 5 consecutive reads match the exact same colour.
 */
DetectedColour_t ColourSensor_GetColour(void)
{
    DetectedColour_t firstSample = evaluateSingleSample();

    // If the first sample is unknown, skip further checks to save processing time
    if (firstSample == COLOUR_UNKNOWN)
    {
        return COLOUR_UNKNOWN;
    }

    // Repeat 5 times
    for (int i = 1; i < 5; i++)
    {
        HAL_Delay(5); // Small delay between consecutive scans
        DetectedColour_t nextSample = evaluateSingleSample();

        if (nextSample != firstSample)
        {
            return COLOUR_UNKNOWN;
        }
    }

    return firstSample;
}
