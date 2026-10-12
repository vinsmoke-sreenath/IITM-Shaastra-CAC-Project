/**
 ******************************************************************************
 * @file    servo.c
 * @brief   Servo control
 ******************************************************************************
 */
#include "servo.h"

/* ---- Private configuration ------------------------------------------------ */
#define SERVO_MIN_PULSE_US        500     /* pulse at 0 deg         */
#define SERVO_MAX_PULSE_US        2500    /* pulse at max angle     */

#define SERVO_S12_MAX_ANGLE       180
#define SERVO_S34_MAX_ANGLE       270

#define SERVO_S12_DEFAULT_ANGLE   180
#define SERVO_S34_DEFAULT_ANGLE   52

/* ---- Private state -------------------------------------------------------- */
static TIM_HandleTypeDef *tim_s12;
static TIM_HandleTypeDef *tim_s34;

/* Last commanded angles. Nothing reads them yet; kept for the debugger. */
static volatile int16_t s12_angle = SERVO_S12_DEFAULT_ANGLE;
static volatile int16_t s34_angle = SERVO_S34_DEFAULT_ANGLE;

/* ---- Private helpers ------------------------------------------------------ */
/*
 * Both servo pairs work the same way:
 *   CH1 pulse = 500 us + angle * 2000 / max_angle
 *   CH2 pulse = 2500 us - angle * 2000 / max_angle     (mirrored servo)
 * The timer tick is 1 us, so the pulse width in us is the compare value.
 * Returns the clamped angle that was applied.
 */
static int set_servo_pair(TIM_HandleTypeDef *htim, int angle, int max_angle)
{
    if (angle < 0)         angle = 0;
    if (angle > max_angle) angle = max_angle;

    uint32_t offset = (uint32_t)((angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / max_angle);

    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, SERVO_MIN_PULSE_US + offset);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, SERVO_MAX_PULSE_US - offset);

    return angle;
}

/* ---- Public API ----------------------------------------------------------- */
void Servo_Init(TIM_HandleTypeDef *htim_s12, TIM_HandleTypeDef *htim_s34)
{
    tim_s12 = htim_s12;
    tim_s34 = htim_s34;

    HAL_TIM_PWM_Start(tim_s12, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(tim_s12, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(tim_s34, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(tim_s34, TIM_CHANNEL_2);

    Servo_Set_S12(SERVO_S12_DEFAULT_ANGLE);
    Servo_Set_S34(SERVO_S34_DEFAULT_ANGLE);
}

void Servo_Set_S12(int angle)
{
    s12_angle = (int16_t)set_servo_pair(tim_s12, angle, SERVO_S12_MAX_ANGLE);
}

void Servo_Set_S34(int angle)
{
    s34_angle = (int16_t)set_servo_pair(tim_s34, angle, SERVO_S34_MAX_ANGLE);
}
