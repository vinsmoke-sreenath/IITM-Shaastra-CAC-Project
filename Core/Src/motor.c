/**
 ******************************************************************************
 * @file    motor.c
 * @brief   DC motor PWM and direction control
 ******************************************************************************
 */
#include "motor.h"
#include <stdlib.h>   /* abs() */

/* ---- Private configuration ------------------------------------------------ */
/* Must equal the Period (ARR) of TIM1 / TIM5 / TIM8 in CubeMX. */
#define MOTOR_PWM_PERIOD   999

#define LEFT_FWD_CHANNEL   TIM_CHANNEL_3   /* on htim_fwd       */
#define RIGHT_FWD_CHANNEL  TIM_CHANNEL_4   /* on htim_fwd       */
#define LEFT_REV_CHANNEL   TIM_CHANNEL_3   /* on htim_left_rev  */
#define RIGHT_REV_CHANNEL  TIM_CHANNEL_3   /* on htim_right_rev */

/* ---- Private state -------------------------------------------------------- */
static TIM_HandleTypeDef *tim_fwd;
static TIM_HandleTypeDef *tim_left_rev;
static TIM_HandleTypeDef *tim_right_rev;

/* ---- Private helpers ------------------------------------------------------ */
static int clamp_speed(int speed)
{
    if (speed >  MOTOR_MAX_SPEED) return  MOTOR_MAX_SPEED;
    if (speed < -MOTOR_MAX_SPEED) return -MOTOR_MAX_SPEED;
    return speed;
}

/* Drive one motor: one PWM channel for forward, one for reverse. */
static void drive_motor(TIM_HandleTypeDef *fwd_tim, uint32_t fwd_ch,
                        TIM_HandleTypeDef *rev_tim, uint32_t rev_ch,
                        int speed)
{
    uint32_t pwm = ((uint32_t)abs(speed) * MOTOR_PWM_PERIOD) / MOTOR_MAX_SPEED;

    if (speed > 0)
    {
        __HAL_TIM_SET_COMPARE(fwd_tim, fwd_ch, pwm);
        __HAL_TIM_SET_COMPARE(rev_tim, rev_ch, 0);
    }
    else if (speed < 0)
    {
        __HAL_TIM_SET_COMPARE(fwd_tim, fwd_ch, 0);
        __HAL_TIM_SET_COMPARE(rev_tim, rev_ch, pwm);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(fwd_tim, fwd_ch, 0);
        __HAL_TIM_SET_COMPARE(rev_tim, rev_ch, 0);
    }
}

/* ---- Public API ----------------------------------------------------------- */
void Motor_Init(TIM_HandleTypeDef *htim_fwd,
                TIM_HandleTypeDef *htim_left_rev,
                TIM_HandleTypeDef *htim_right_rev)
{
    tim_fwd       = htim_fwd;
    tim_left_rev  = htim_left_rev;
    tim_right_rev = htim_right_rev;

    HAL_TIM_PWM_Start(tim_fwd,       LEFT_FWD_CHANNEL);
    HAL_TIM_PWM_Start(tim_fwd,       RIGHT_FWD_CHANNEL);
    HAL_TIM_PWM_Start(tim_left_rev,  LEFT_REV_CHANNEL);
    HAL_TIM_PWM_Start(tim_right_rev, RIGHT_REV_CHANNEL);

    Motor_Set_Speeds(0, 0);
}

void Motor_Set_Speeds(int left_speed, int right_speed)
{
    left_speed  = clamp_speed(left_speed);
    right_speed = clamp_speed(right_speed);

    drive_motor(tim_fwd, LEFT_FWD_CHANNEL,  tim_left_rev,  LEFT_REV_CHANNEL,  left_speed);
    drive_motor(tim_fwd, RIGHT_FWD_CHANNEL, tim_right_rev, RIGHT_REV_CHANNEL, right_speed);
}
