/**
 ******************************************************************************
 * @file    uart_comm.h
 * @brief   UART link to the Raspberry Pi (public API)
 *
 * Owns    : UART5 reception, command parsing, RPM telemetry transmission
 * Calls   : pid.h   (VEL command)   servo.h (S12 / S34 commands)
 * Used by : main.c
 *
 * Commands received (terminated by \n or \r):
 *     VEL,left_rpm,right_rpm     e.g. VEL,100,-100
 *     S12:angle                  e.g. S12:90
 *     S34:angle                  e.g. S34:135
 *
 * Telemetry sent:
 *     RPM,left_rpm,right_rpm     e.g. RPM,98.42,100.17
 *     IMU,ax,ay,az,gx,gy,gz      e.g. IMU,120,-35,8190,12,-4,7   (raw counts)
 *         accel: 8192 counts per g     (MPU6050 set to +-4 g)
 *         gyro : 65.5 counts per deg/s (MPU6050 set to +-500 dps)
 *
 * Robustness:
 *   - Stray bytes in front of a command are skipped ("xVEL,100,100" still works).
 *   - Command timeout: once a VEL command has been received, the targets are
 *     forced to 0,0 if no further valid VEL arrives within 500 ms. The Pi must
 *     therefore repeat its VEL command (keep-alive) while it wants motion.
 *     Call UART_Comm_Check_Timeout() from the main loop to enforce this.
 *
 * The HAL callbacks HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback are
 * implemented in uart_comm.c. They are called by the HAL, not by our code,
 * so they are not listed here.
 ******************************************************************************
 */
#ifndef UART_COMM_H
#define UART_COMM_H

#include "main.h"

/* Start interrupt-driven reception. Handle is the CubeMX-generated huart5. */
HAL_StatusTypeDef UART_Comm_Init(UART_HandleTypeDef *huart);

/* Stop the rover if the Pi has gone silent. Cheap; call on every main-loop pass. */
void UART_Comm_Check_Timeout(void);

/* Send one telemetry line. The caller supplies the values (no dependency on encoder.h). */
void UART_Comm_Send_RPM(float left_rpm, float right_rpm);

/* Send one IMU line with the raw MPU6050 counts. */
void UART_Comm_Send_IMU(int16_t ax, int16_t ay, int16_t az,
                        int16_t gx, int16_t gy, int16_t gz);

#endif /* UART_COMM_H */
