/**
 ******************************************************************************
 * @file    pid.h
 * @brief   Closed-loop wheel RPM control (public API)
 *
 * Owns    : target RPMs, left/right PID controllers, gains, limits
 * Reads   : encoder.h  (actual RPM)
 * Drives  : motor.h    (PWM command)
 * Used by : uart_comm.c (sets targets), main.c (init + periodic update)
 ******************************************************************************
 */
#ifndef PID_H
#define PID_H

#include "main.h"

#define PID_SAMPLE_TIME_MS   100U

void PID_Init(void);

void PID_Set_Target_RPM(float left_rpm, float right_rpm);

void PID_Update(void);

#endif /* PID_H */
