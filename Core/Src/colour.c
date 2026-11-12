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
#include <stdlib.h>

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
	#define TCS34725_GAIN_16X        0x02  /* Gain setting: 4x */

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
	  * @param  hi2c Pointer to your configured I2C handle (e.g., &hi2c1)
	  * @retval HAL_OK if initialization succeeded
	  */
	HAL_StatusTypeDef ColourSensor_Init(I2C_HandleTypeDef *hi2c)
	{
		if (hi2c == NULL) return HAL_ERROR;
		tcs_hi2c = hi2c;

		uint8_t id = 0;
		uint8_t command = TCS34725_COMMAND_BIT | TCS34725_ID;

		// Check Sensor Device ID (TCS34725 returns 0x44 or 0x4D)
		HAL_I2C_Master_Transmit(tcs_hi2c, TCS34725_I2C_ADDR, &command, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(tcs_hi2c, TCS34725_I2C_ADDR, &id, 1, HAL_MAX_DELAY);

		if (id != 0x44 && id != 0x4D)
		{
			return HAL_ERROR; // ID mismatch
		}

		// Set integration time & gain
		writeRegister(TCS34725_ATIME, TCS34725_ATIME_154MS);
		writeRegister(TCS34725_CONTROL, TCS34725_GAIN_16X);

		// Power ON and Enable ADC
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
	  * @brief  Algorithm to classify colour into RED, GREEN, or BLUE
	  * @retval DetectedColour_t Enum value representing the colour
	  */



	/*DetectedColour_t ColourSensor_GetColour(void)
	{
		ColourData_t raw;
		if (ColourSensor_ReadRaw(&raw) != HAL_OK || raw.c == 0)
		{
			return COLOUR_UNKNOWN;
		}

	   // RBG ratio
		float rRatio = (float)raw.r / (float)raw.c;
		float gRatio = (float)raw.g / (float)raw.c;
		float bRatio = (float)raw.b / (float)raw.c;



		//prevent false readings in low light conditions
		if (raw.c < 50)
		{
			return COLOUR_UNKNOWN;
		}

		// Classification Algorithm: Primary Dominance Strategy
		if (rRatio > gRatio && rRatio > bRatio && rRatio > 0.40f)
		{
			return COLOUR_RED;
		}
		else if (gRatio > rRatio && gRatio > bRatio && gRatio > 0.38f)
		{
			return COLOUR_GREEN;
		}
		else if (bRatio > rRatio && bRatio > gRatio && bRatio > 0.35f)
		{
			return COLOUR_BLUE;
		}

		return COLOUR_UNKNOWN;
	}

*/

	DetectedColour_t ColourSensor_GetColour(void)
	{
	    ColourData_t raw;
	    if (ColourSensor_ReadRaw(&raw) != HAL_OK)
	    {
	        btPrint("Read Failed\n");
	        return COLOUR_UNKNOWN;
	    }

	    // Print raw values to your Bluetooth terminal for debugging
	    char msg[64];
	    sprintf(msg, "C:%d R:%d G:%d B:%d\r\n", raw.c, raw.r, raw.g, raw.b);
	    btPrint(msg);

	    // Prevent false readings in low light conditions
	    if (raw.c < 50)
	    {
	        return COLOUR_UNKNOWN;
	    }

	    // Calculate RGB ratios
	    float rRatio = (float)raw.r / (float)raw.c;
	    float gRatio = (float)raw.g / (float)raw.c;
	    float bRatio = (float)raw.b / (float)raw.c;

	    // Classification Algorithm: Strict RGB Filtering (Ignoring Yellow/Mixed colors)

	    // Red: Red must be dominant AND at least 12% higher than green
	    if (rRatio > (gRatio + 0.18f) && rRatio > bRatio && rRatio > 0.40f)
	    {
	        return COLOUR_RED;
	    }
	    // Green: Green must be dominant AND at least 10% higher than red
	    // Green check
	    else if (gRatio > rRatio && gRatio > bRatio && gRatio > 0.35f)
	    {
	        return COLOUR_GREEN;
	    }
	    // Blue check: Blue must clearly dominate both
	    else if (bRatio > 0.20f && bRatio < 0.35f && abs(rRatio - gRatio) < 0.05f && raw.c > 100)
	        {
	            return COLOUR_BLUE;
	        }

	    return COLOUR_UNKNOWN;
	}
