/**
 ******************************************************************************
 * @file    encoder.c
 * @brief   Wheel encoders: tick counting and RPM calculation
 ******************************************************************************
 */
#include "encoder.h"

/* ---- Private configuration ------------------------------------------------ */
#define ENCODER_PPR           11
#define ENCODER_GEAR_RATIO    30
#define ENCODER_TOTAL_TICKS   (ENCODER_PPR * ENCODER_GEAR_RATIO * 4)  /* x4: TI12 mode */

/* ---- Private state -------------------------------------------------------- */
typedef struct
{
    TIM_HandleTypeDef *htim;        /* encoder timer                           */
    uint16_t           prev_count;  /* counter value at the previous sample    */
    float              rpm;         /* result of the last Encoder_Update()     */
} Encoder_Channel;

static Encoder_Channel left_enc;
static Encoder_Channel right_enc;

/* ---- Private helpers (left and right share the same code) ----------------- */
static void channel_start(Encoder_Channel *ch, TIM_HandleTypeDef *htim)
{
    ch->htim = htim;
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    ch->prev_count = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
    ch->rpm = 0.0f;
}

static void channel_update(Encoder_Channel *ch)
{
    uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(ch->htim);

    /*
     * Unsigned 16-bit subtraction cast to int16_t handles counter wrap-around
     * (65535 -> 0 or 0 -> 65535) and gives a signed tick count (direction).
     */
    int16_t ticks = (int16_t)(current - ch->prev_count);
    ch->prev_count = current;

    /* rpm = (ticks / ticks_per_rev) * (60000 ms / sample_time_ms) */
    ch->rpm = ((float)ticks / (float)ENCODER_TOTAL_TICKS)
            * (60000.0f / ENCODER_SAMPLE_TIME_MS);
}

/* ---- Public API ----------------------------------------------------------- */
void Encoder_Init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right)
{
    channel_start(&left_enc,  htim_left);
    channel_start(&right_enc, htim_right);
}

void Encoder_Update(void)
{
    channel_update(&left_enc);
    channel_update(&right_enc);
}

float Encoder_Get_Left_RPM(void)
{
    return left_enc.rpm;
}

float Encoder_Get_Right_RPM(void)
{
    return right_enc.rpm;
}
