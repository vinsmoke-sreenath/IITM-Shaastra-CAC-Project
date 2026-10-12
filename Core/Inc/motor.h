/**
 ******************************************************************************
 * @file    motor.h
 * @brief   DC motor PWM and direction control (public API)
 *
 * Owns    : motor PWM channels, speed -> PWM conversion, direction selection
 * Used by : pid.c (output), main.c (safe stop on fatal init error)
 *
 * Hardware mapping (private to motor.c):
 *   Left  motor : forward = TIM1 CH3, reverse = TIM5 CH3
 *   Right motor : forward = TIM1 CH4, reverse = TIM8 CH3
 ******************************************************************************
 */
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

#define MOTOR_MAX_SPEED   500

void Motor_Init(TIM_HandleTypeDef *htim_fwd,
                TIM_HandleTypeDef *htim_left_rev,
                TIM_HandleTypeDef *htim_right_rev);

void Motor_Set_Speeds(int left_speed, int right_speed);

#endif /* MOTOR_H */
