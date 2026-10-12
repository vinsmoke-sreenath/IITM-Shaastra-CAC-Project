/**
 ******************************************************************************
 * @file    encoder.h
 * @brief   Wheel encoders: tick counting and RPM calculation (public API)
 *
 * Owns    : TIM2 (left) / TIM4 (right) encoder counters, tick -> RPM maths
 * Used by : pid.c  (feedback)      main.c (telemetry)
 ******************************************************************************
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

#define ENCODER_SAMPLE_TIME_MS   100U

void  Encoder_Init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right);
void  Encoder_Update(void);
float Encoder_Get_Left_RPM(void);
float Encoder_Get_Right_RPM(void);

#endif /* ENCODER_H */
