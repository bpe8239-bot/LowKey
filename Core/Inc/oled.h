/*
 * oled.h
 *
 *  Created on: 20 Nov 2026
 *      Author: bpe82
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

/* Application-level OLED UI for STM32F103 / SSD1306 on I2C2.
 * Requires the afiskon stm32-ssd1306 library configured for hi2c2.
 * Call all functions from the main loop, not from interrupt handlers.
 */

void OLED_UI_Init(void);
void OLED_UI_SetStatus(const char *status);            /* continuous navigation/debug status */
void OLED_UI_EventStatus(const char *event);           /* e.g. CORNER SEEN, BRANCH DETECTED */
void OLED_UI_SetMode(const char *mode);                /* popup + mode label */
void OLED_UI_Claw(const char *operation);              /* popup, e.g. OPEN / CLOSE */
void OLED_UI_Tick(int pwm);                            /* call once each main-loop pass */
void OLED_UI_ColourDetected(const char *colour);

#endif /* INC_OLED_H_ */
