/**
 ******************************************************************************
 * @file    servo.h
 * @brief   Servo control (public API)
 *
 * Owns    : servo PWM channels, angle -> pulse-width conversion, defaults
 * Used by : uart_comm.c (S12 / S34 commands)
 *
 * Servo pair S1/S2 : TIM3 CH1/CH2, 0..180 deg  (S2 mirrored)
 * Servo pair S3/S4 : TIM9 CH1/CH2, 0..270 deg  (S4 mirrored)
 ******************************************************************************
 */
#ifndef SERVO_H
#define SERVO_H

#include "main.h"

void Servo_Init(TIM_HandleTypeDef *htim_s12, TIM_HandleTypeDef *htim_s34);

void Servo_Set_S12(int angle);   /* 0..180 */
void Servo_Set_S34(int angle);   /* 0..270 */

#endif /* SERVO_H */
