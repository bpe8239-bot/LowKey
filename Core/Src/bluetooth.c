/*  Created on: 7 Nov 2026
 *      Author: bpe82
 */
#include "bluetooth.h"
#include <string.h>

#define BT_QUEUE_SIZE      32U

static uint8_t rxByte; /* ISR writes to the queue; the main loop reads from it. */
static volatile uint8_t btQueue[BT_QUEUE_SIZE];
static volatile uint8_t btHead = 0;
static volatile uint8_t btTail = 0;

extern UART_HandleTypeDef huart1;

/* --------- Bluetooth serial (USART1 -> BT-06 -> phone) --------- */
/* Call only from the main loop, never from the UART receive interrupt. */
void btPrint(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 150);
}

uint8_t btPop(uint8_t *result)
{
    if (btTail == btHead) return 0;
    *result = btQueue[btTail];
    btTail = (uint8_t)((btTail + 1U) % BT_QUEUE_SIZE);
    return 1;
}


/* IRQ: buffer received bytes and restart reception. No blocking print here. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		const uint8_t next = (uint8_t)((btHead + 1U) % BT_QUEUE_SIZE);
		if (next != btTail) /* silently discard byte if buffer is full */
		{
			btQueue[btHead] = rxByte;
			btHead = next;
		}
		HAL_UART_Receive_IT(&huart1, &rxByte, 1);
	}
}

//In case of UART error
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (huart->RxState == HAL_UART_STATE_READY)
        {
            HAL_UART_Receive_IT(huart, &rxByte, 1);
        }
    }
}

HAL_StatusTypeDef Bluetooth_Start(void)
{
    return HAL_UART_Receive_IT(&huart1, &rxByte, 1);
}
