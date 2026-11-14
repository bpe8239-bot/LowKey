/* USER CODE BEGIN Header */
/* Blue Pill (STM32F103C8T6) + mini L298N (no ENA/ENB pins) + Bluetooth BT-06
 * WASD control over Bluetooth serial. STM32CubeIDE + HAL.
 *
 * HOW THE MINI L298N IS DRIVEN (no enable pins):
 *   Speed is done by putting PWM directly on the IN pins.
 *     forward : IN_A = PWM, IN_B = 0
 *     reverse : IN_A = 0,   IN_B = PWM
 *     stop    : both 0
 *
 * ---------- WIRING ----------
 *   PA0 -> IN1   (left motor)
 *   PA1 -> IN2   (left motor)
 *   PA2 -> IN3   (right motor)
 *   PA3 -> IN4   (right motor)
 *   Motor driver GND -> Blue Pill GND (common ground)
 *   Motor battery    -> driver motor power input
 *
 *   BT-06 TXD -> PA10 (USART1 RX)
 *   BT-06 RXD -> PA9  (USART1 TX)
 *   BT-06 VCC -> 5V,  BT-06 GND -> GND
 *
 * ---------- CubeMX SETTINGS (must match for this file to work) ----------
 *   SYS    : Debug = Serial Wire
 *   RCC    : HSE = Crystal/Ceramic Resonator, 72 MHz clock (HCLK 72, APB1 36)
 *   TIM2   : Clock Source = Internal Clock
 *            CH1, CH2, CH3, CH4 = PWM Generation  (PA0, PA1, PA2, PA3)
 *            Prescaler = 71, Counter Period = 999   (1 kHz PWM, duty 0-999)
 *   USART1 : Asynchronous, 9600 baud, 8N1 (PA9 TX, PA10 RX)
 *            NVIC: USART1 global interrupt = Enabled
 *   Project Manager: generate code, then replace main.c with this file.
 *
 * ---------- KEYS (send one character via a Bluetooth serial terminal) ----------
 *   W = forward     S = backward     A = turn left     D = turn right
 *   X = stop
 *   1-9 = speed (1 slowest, 9 fastest)
 *   ? = show help. Manual movement times out after 1 seconds without a new command.
 *   N = Line Following ONLY
 *   L = Line Logging
 *   M = Basic Map Loop
 *   T = Show KP KD value
 */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "bluetooth.h"
#include "colour.h"
#include "line_following.h"
#include "mission.h"
#include "motor.h"
#include "servo_claw.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_TIMEOUT_MS     1000U /* auto-stop manual movement after 1 s */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static char cmd = 'X';
static int speed = 750;                 /* 0-999 */
static uint32_t lastCmdTime = 0;
static uint8_t autoMode = 0;           /* 1 = Basic Map Loop; 2 = Line Follow only */
static uint8_t colourReady = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Colour Debug
#define COLOUR_DEBUG 0
#define COLOUR_REPORT_MS 500U

static void colourDebugTick(void)
{
#if COLOUR_DEBUG
    static uint32_t lastReport = 0;
    uint32_t now = HAL_GetTick();

    if (!colourReady) return;

    if ((now - lastReport) < COLOUR_REPORT_MS)
        return;

    lastReport = now;

    ColourData_t raw;

    if (ColourSensor_ReadRaw(&raw) != HAL_OK)
    {
        btPrint("[COLOUR] READ ERROR\r\n");
        return;
    }

    DetectedColour_t detected = ColourSensor_GetColour();

    const char *name = "UNKNOWN";

    if (detected == COLOUR_RED)
        name = "RED";
    else if (detected == COLOUR_GREEN)
        name = "GREEN";
    else if (detected == COLOUR_BLUE)
        name = "BLUE";

    char msg[100];

    snprintf(msg, sizeof(msg),
             "[COLOUR] R:%u G:%u B:%u C:%u | %s\r\n",
             (unsigned)raw.r,
             (unsigned)raw.g,
             (unsigned)raw.b,
             (unsigned)raw.c,
             name);

    btPrint(msg);
#endif
}

/*Bluetooth Keybind command*/
static void handleBtCommand(char c)
{
    static char rx_buf[32];
    static uint8_t rx_idx = 0;
    static uint8_t debugMode = 0; // 0 = OFF (Locked), 1 = ON (Unlocked)
    const char *motion = NULL;
    char line[90];

    // 1. Check for end of line (Enter / Newline sent from phone app)
    if (c == '\r' || c == '\n')
    {
        if (rx_idx == 0) return; // Ignore empty newlines
        rx_buf[rx_idx] = '\0';   // Null-terminate string
        rx_idx = 0;              // Reset buffer index for next command

        // Block multi-character tuning/status commands if Debug Mode is OFF
        if (!debugMode)
        {
            btPrint("[LOCKED] Send 'T' to enable Debug Mode first\r\n");
            return;
        }

        char prefix = rx_buf[0];

        // --- Status Command ('v' or 'V') ---
        if (prefix == 'v' || prefix == 'V')
        {
            char msg[120];
            sprintf(msg, "[STATUS] Margin:%.2f | Rmin:%.2f | Gmin:%.2f | Bmin:%.2f | Bmax:%.2f | RGdiff:%.2f\r\n",
                    tuning_red_margin, tuning_red_min, tuning_green_min, tuning_blue_min, tuning_blue_max, tuning_rg_diff);
            btPrint(msg);
            return;
        }

        // --- Direct Value Tuning (e.g. m0.10, o0.62, r0.40) ---
        char *endptr = NULL;
        float val = strtof(&rx_buf[1], &endptr);
        if (endptr != &rx_buf[1] && *endptr == '\0')
        {
            char msg[60];
            switch (prefix)
            {
                case 'm': case 'M':
                    tuning_red_margin = val;
                    sprintf(msg, "[TUNE] Red Margin set to %.2f\r\n", tuning_red_margin);
                    break;
                case 'r': case 'R':
                    tuning_red_min = val;
                    sprintf(msg, "[TUNE] Red Min set to %.2f\r\n", tuning_red_min);
                    break;
                case 'g': case 'G':
                    tuning_green_min = val;
                    sprintf(msg, "[TUNE] Green Min set to %.2f\r\n", tuning_green_min);
                    break;
                case 'b': case 'B':
                    tuning_blue_min = val;
                    sprintf(msg, "[TUNE] Blue Min set to %.2f\r\n", tuning_blue_min);
                    break;
                case 'o': case 'O':
                    tuning_blue_max = val;
                    sprintf(msg, "[TUNE] Blue Max set to %.2f\r\n", tuning_blue_max);
                    break;
                case 'p': case 'P':
                    tuning_rg_diff = val;
                    sprintf(msg, "[TUNE] RG Diff set to %.2f\r\n", tuning_rg_diff);
                    break;
                default:
                    btPrint("[ERR] Unknown letter prefix\r\n");
                    return;
            }
            btPrint(msg);
            return;
        }

        btPrint("[ERR] Format error. Use letter + value (e.g., m0.10, o0.62)\r\n");
        return;
    }

    // 2. Accumulate normal characters into buffer until newline arrives
    if (rx_idx < sizeof(rx_buf) - 1)
    {
        rx_buf[rx_idx++] = c;
    }
    else
    {
        rx_idx = 0; // Overflow safety reset
        btPrint("[ERR] Command string too long\r\n");
        return;
    }

    // 3. Instant Single-Character Commands
    if (rx_idx == 1)
    {
        // --- Explicit Debug Mode Control ('T' = ON, 't' = OFF) ---
        if (c == 'T')
        {
            debugMode = 1;
            btPrint("[DEBUG] ON - PID & Colour tuning unlocked\r\n");
            rx_idx = 0;
            return;
        }
        if (c == 't')
        {
            debugMode = 0;
            btPrint("[DEBUG] OFF - Tuning locked\r\n");
            rx_idx = 0;
            return;
        }

        // --- Debug-Only Single-Char Commands (PID Tuning, Color Cal, Color Test) ---
        if (debugMode)
        {
            if (pidTuneKey(c)) { rx_idx = 0; return; }

            if (c == 'c' || c == 'C')
            {
                ColourSensor_CalibrateWhite();
                btPrint("[CAL] White Calibrated\r\n");
                rx_idx = 0;
                return;
            }

            if (c == 'z' || c == 'Z')
            {
                ColourData_t raw;
                if (ColourSensor_ReadRaw(&raw) == HAL_OK)
                {
                    float rR = (raw.c > 0) ? (float)raw.r / raw.c : 0;
                    float gR = (raw.c > 0) ? (float)raw.g / raw.c : 0;
                    float bR = (raw.c > 0) ? (float)raw.b / raw.c : 0;
                    char msg[80];
                    sprintf(msg, "[RATIOS] R:%.2f G:%.2f B:%.2f C:%u\r\n", rR, gR, bR, raw.c);
                    btPrint(msg);

                    DetectedColour_t det = ColourSensor_GetColour();
                    if (det == COLOUR_RED) btPrint("-> RED\r\n");
                    else if (det == COLOUR_GREEN) btPrint("-> GREEN\r\n");
                    else if (det == COLOUR_BLUE) btPrint("-> BLUE\r\n");
                    else btPrint("-> UNKNOWN\r\n");
                }
                rx_idx = 0;
                return;
            }
        }
        else
        {
            if (c == 'c' || c == 'C' || c == 'z' || c == 'Z')
            {
                btPrint("[LOCKED] Send 'T' to enable Debug Mode\r\n");
                rx_idx = 0;
                return;
            }
        }

        // --- Standard Driving & Mission Commands (Always Available) ---
        if (c >= '1' && c <= '9')
        {
            speed = 300 + (c - '1') * 87;  /* 1..9 -> 300..996 */
            lastCmdTime = HAL_GetTick();
            snprintf(line, sizeof(line), "[SPEED] PWM = %d / 999\r\n", speed);
            btPrint(line);
            rx_idx = 0;
            return;
        }
        if (c == 'M') { autoMode = 1; missionStart(); btPrint("[MODE] MISSION STARTS\r\n"); rx_idx = 0; return; }
        if (c == 'N' || c == 'n') { autoMode = 2; lineFollowReset(); btPrint("[MODE] LINE FOLLOW ONLY\r\n"); rx_idx = 0; return; }
        if (c == 'X' || c == 'x' || c == ' ') { autoMode = 0; cmd = 'X'; stopMotors(); btPrint("[MOTOR] STOP\r\n"); rx_idx = 0; return; }
        if (c == '?' || c == 'H' || c == 'h') { btPrint("[HELP] W/S/A/D move; X stop; 1-9 speed; T/t debug on/off; (Debug: m/r/g/b/o/p+val, v, c, z)\r\n"); rx_idx = 0; return; }
        if (c == 'F' || c == 'f') { Claw_Open(); btPrint("[CLAW] OPEN\r\n"); rx_idx = 0; return; }
        if (c == 'G') { Claw_Close(); btPrint("[CLAW] CLOSE\r\n"); rx_idx = 0; return; }
        if (c == 'L' || c == 'l') { autoMode = 0; cmd = 'X'; stopMotors(); lineLogDump(); rx_idx = 0; return; }

        // Movement (W/A/S/D) - executes immediately on keystroke
        if (c == 'W' || c == 'w') motion = "FORWARD";
        else if (c == 'S' || c == 's') motion = "BACKWARD";
        else if (c == 'A' || c == 'a') motion = "LEFT";
        else if (c == 'D' || c == 'd') motion = "RIGHT";

        if (motion != NULL)
        {
            autoMode = 0;
            cmd = c;
            lastCmdTime = HAL_GetTick();
            snprintf(line, sizeof(line), "[MOTOR] %s | Speed: %d\r\n", motion, speed);
            btPrint(line);
            rx_idx = 0;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  stopMotors();

    lastCmdTime = HAL_GetTick();
    if (Bluetooth_Start() != HAL_OK)
    {
        Error_Handler();
    }

    /* start listening to Bluetooth */
  btPrint("[STM32] Ready. Send ? for help.\r\n");

  	//Claw
    if (Claw_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    //Colour Sensor
    if (ColourSensor_Init(&hi2c1) == HAL_OK)
    {
        colourReady = 1;
        btPrint("[COLOUR] INIT OK\r\n");
    }
    else
    {
        btPrint("[COLOUR] INIT FAILED\r\n");
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t received;
    while (btPop(&received))
    {
      handleBtCommand((char)received);
    }
    lineLogTick();

    if (autoMode == 1) //Basic Map Loop
        {
          if (!missionTick(speed))
          {
            autoMode = 0;
            cmd = 'X';
            btPrint("[MODE] MANUAL - W/A/S/D drive, F/G claw\r\n");
          }
        }
    else if (autoMode == 2) //Line Following
    {
    	lineFollow(speed);
    }
    else
    {
      if (cmd != 'X' && cmd != 'x' && cmd != ' ' &&
          HAL_GetTick() - lastCmdTime >= CMD_TIMEOUT_MS)
      {
        cmd = 'X';
        stopMotors();
        btPrint("[SAFETY] Manual command timeout: STOP\r\n");
      }
      switch (cmd)
      {
        case 'W': case 'w': forward(speed);   break;
        case 'S': case 's': backward(speed);  break;
        case 'A': case 'a': turnLeft(speed);  break;
        case 'D': case 'd': turnRight(speed); break;
        default: stopMotors(); break;
      }
      colourDebugTick();
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
