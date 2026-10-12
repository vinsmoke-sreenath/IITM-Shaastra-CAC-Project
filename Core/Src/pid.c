/**
 ******************************************************************************
 * @file    pid.c
 * @brief   Closed-loop wheel RPM control
 ******************************************************************************
 */
#include "pid.h"
#include "encoder.h"   /* actual RPM   */
#include "motor.h"     /* PWM output   */

/* ---- Private configuration ------------------------------------------------ */
#define PID_DT           ((float)PID_SAMPLE_TIME_MS / 1000.0f)   /* = 0.1 s */

#define LEFT_KP          2.0f
#define LEFT_KI          5.0f
#define LEFT_KD          0.0f

#define RIGHT_KP         2.0f
#define RIGHT_KI         5.0f
#define RIGHT_KD         0.0f

#define MAX_TARGET_RPM   500.0f

/* ---- Private types -------------------------------------------------------- */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} PID;

/* ---- Private state -------------------------------------------------------- */
static PID left_pid;
static PID right_pid;

/*
 * Written from the UART RX interrupt (PID_Set_Target_RPM),
 * read from the main loop (PID_Update)  ->  volatile.
 */
static volatile float target_left_rpm  = 0.0f;
static volatile float target_right_rpm = 0.0f;

/* ---- Private: PID algorithm (unchanged from the original code) ------------ */
static void pid_init(PID *pid, float Kp, float Ki, float Kd,
                     float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

static void pid_reset(PID *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static float pid_update(PID *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f)
    {
        return 0.0f;
    }

    float error = setpoint - measurement;

    /* Proportional */
    float P = pid->Kp * error;

    /* Integral: removes steady-state speed error */
    pid->integral += error * dt;
    float I = pid->Ki * pid->integral;

    /* Derivative: Kd is 0 today, kept so it can be enabled later */
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->Kd * derivative;
    pid->prev_error = error;

    float output = P + I + D;

    /* Output limiting + anti-windup (undo this step's integral when saturated) */
    if (output > pid->out_max)
    {
        output = pid->out_max;
        pid->integral -= error * dt;
    }
    else if (output < pid->out_min)
    {
        output = pid->out_min;
        pid->integral -= error * dt;
    }

    return output;
}

/* ---- Private: target handling --------------------------------------------- */
static float clamp_target(float rpm)
{
    if (rpm >  MAX_TARGET_RPM) return  MAX_TARGET_RPM;
    if (rpm < -MAX_TARGET_RPM) return -MAX_TARGET_RPM;
    return rpm;
}

/* True when the wheel is asked to reverse (+ -> - or - -> +). */
static int direction_changed(float old_target, float new_target)
{
    return (old_target > 0.0f && new_target < 0.0f) ||
           (old_target < 0.0f && new_target > 0.0f);
}

/* ---- Public API ----------------------------------------------------------- */
void PID_Init(void)
{
    target_left_rpm  = 0.0f;
    target_right_rpm = 0.0f;

    pid_init(&left_pid,  LEFT_KP,  LEFT_KI,  LEFT_KD,  -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    pid_init(&right_pid, RIGHT_KP, RIGHT_KI, RIGHT_KD, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
}

void PID_Set_Target_RPM(float left_rpm, float right_rpm)
{
    left_rpm  = clamp_target(left_rpm);
    right_rpm = clamp_target(right_rpm);

    /* Reversing direction: drop the old integral so it cannot fight the new direction */
    if (direction_changed(target_left_rpm, left_rpm))
    {
        pid_reset(&left_pid);
    }
    if (direction_changed(target_right_rpm, right_rpm))
    {
        pid_reset(&right_pid);
    }

    target_left_rpm  = left_rpm;
    target_right_rpm = right_rpm;

    /*
     * Immediate stop of a wheel whose target is 0, without waiting for the
     * next PID tick.
     *
     * NOTE: carried over unchanged from the original code. The other wheel
     * is written with its target RPM used as a speed command (open loop)
     * until the next PID_Update(). See the review notes.
     */
    if (left_rpm == 0.0f)
    {
        pid_reset(&left_pid);
        Motor_Set_Speeds(0, (int)target_right_rpm);
    }
    if (right_rpm == 0.0f)
    {
        pid_reset(&right_pid);
        Motor_Set_Speeds((int)target_left_rpm, 0);
    }
}

void PID_Update(void)
{
    float left_output;
    float right_output;

    /*
     * PID_Set_Target_RPM() runs in the UART interrupt and also resets the
     * controllers. Block interrupts for the few microseconds the maths takes
     * so a command cannot arrive half-way through a control step.
     */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (target_left_rpm == 0.0f)
    {
        pid_reset(&left_pid);
        left_output = 0.0f;
    }
    else
    {
        left_output = pid_update(&left_pid, target_left_rpm,
                                 Encoder_Get_Left_RPM(), PID_DT);
    }

    if (target_right_rpm == 0.0f)
    {
        pid_reset(&right_pid);
        right_output = 0.0f;
    }
    else
    {
        right_output = pid_update(&right_pid, target_right_rpm,
                                  Encoder_Get_Right_RPM(), PID_DT);
    }

    __set_PRIMASK(primask);

    Motor_Set_Speeds((int)left_output, (int)right_output);
}
