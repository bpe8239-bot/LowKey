/*
 * motor.h
 *
 *  Created on: 7 Nov 2026
 *      Author: bpe82
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

void setLeft(int spd);
void setRight(int spd);

void forward(int s);
void backward(int s);
void turnLeft(int s);
void turnRight(int s);
void stopMotors(void);

#endif /* INC_MOTOR_H_ */
