/**
 ******************************************************************************
 * @file    uart_comm.c
 * @brief   UART link to the Raspberry Pi: command parsing and RPM telemetry
 ******************************************************************************
 */
#include "uart.h"
#include "pid.h"
#include "servo.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* ---- Private configuration ------------------------------------------------ */
#define RX_BUFFER_SIZE       64
#define RPM_TX_BUFFER_SIZE   48
#define IMU_TX_BUFFER_SIZE   64     /* worst case line is 47 characters */
#define TX_TIMEOUT_MS        10
#define VEL_TIMEOUT_MS       500U   /* stop if no valid VEL for this long */

/* ---- Private state -------------------------------------------------------- */
static UART_HandleTypeDef *uart;

/* rx_byte is filled by the HAL through a pointer, hence volatile. */
static volatile uint8_t rx_byte;

/* Only ever touched inside the RX interrupt, so no volatile needed. */
static char    rx_buffer[RX_BUFFER_SIZE];
static uint8_t rx_index = 0;

/* Last servo command text. Write-only today; kept for the debugger. */
static char last_servo_cmd[32] = "NONE";

static volatile uint32_t tx_error_count = 0;

/* Command timeout. Written in the RX interrupt, read in the main loop. */
static volatile uint32_t last_vel_tick = 0;
static volatile bool     vel_armed     = false;   /* true after a VEL, until the timeout fires */

/* ---- Private: command parsing --------------------------------------------- */
static void remember_servo_cmd(const char *cmd)
{
    strncpy(last_servo_cmd, cmd, sizeof(last_servo_cmd) - 1);
    last_servo_cmd[sizeof(last_servo_cmd) - 1] = '\0';
}

/*
 * Returns a pointer to the first command keyword in the line, or NULL.
 * Lets a command survive stray bytes in front of it ("xVEL,100,100").
 */
static char *find_command_start(char *line)
{
    static const char *const keywords[] = { "VEL,", "S12:", "S34:" };
    char *best = NULL;

    for (unsigned i = 0; i < sizeof(keywords) / sizeof(keywords[0]); i++)
    {
        char *p = strstr(line, keywords[i]);
        if ((p != NULL) && ((best == NULL) || (p < best)))
        {
            best = p;
        }
    }
    return best;
}

static void process_command(char *line)
{
    int left  = 0;
    int right = 0;
    int angle = 0;

    char *cmd = find_command_start(line);
    if (cmd == NULL)
    {
        return;
    }

    if (strncmp(cmd, "VEL,", 4) == 0)
    {
        /* VEL carries TARGET RPM. Range limiting is the PID module's job. */
        if (sscanf(cmd + 4, "%d,%d", &left, &right) == 2)
        {
            last_vel_tick = HAL_GetTick();
            vel_armed     = true;
            PID_Set_Target_RPM((float)left, (float)right);
        }
    }
    else if (strncmp(cmd, "S12:", 4) == 0)
    {
        if (sscanf(cmd + 4, "%d", &angle) == 1)
        {
            remember_servo_cmd(cmd);
            Servo_Set_S12(angle);
        }
    }
    else if (strncmp(cmd, "S34:", 4) == 0)
    {
        if (sscanf(cmd + 4, "%d", &angle) == 1)
        {
            remember_servo_cmd(cmd);
            Servo_Set_S34(angle);
        }
    }
}

/* ---- Private: telemetry formatting ---------------------------------------- */
/* Prints value with 2 decimals without needing float support in printf. */
static void format_float2(float value, char *out_str)
{
    int is_negative = 0;

    if (value < 0.0f)
    {
        is_negative = 1;
        value = -value;
    }

    int32_t scaled = (int32_t)(value * 100.0f + 0.5f);   /* 123.456 -> 12346 */
    int32_t whole  = scaled / 100;
    int32_t frac   = scaled % 100;

    /* Avoid printing "-0.00" */
    if (is_negative && (whole != 0 || frac != 0))
    {
        sprintf(out_str, "-%ld.%02ld", (long)whole, (long)frac);
    }
    else
    {
        sprintf(out_str, "%ld.%02ld", (long)whole, (long)frac);
    }
}

/* ---- Public API ----------------------------------------------------------- */
HAL_StatusTypeDef UART_Comm_Init(UART_HandleTypeDef *huart)
{
    uart     = huart;
    rx_index = 0;

    return HAL_UART_Receive_IT(uart, (uint8_t *)&rx_byte, 1);
}

void UART_Comm_Check_Timeout(void)
{
    /*
     * Interrupts are masked so the RX interrupt cannot deliver a new VEL
     * between the check and the stop.
     */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (vel_armed && ((uint32_t)(HAL_GetTick() - last_vel_tick) > VEL_TIMEOUT_MS))
    {
        vel_armed = false;                 /* fire once, re-armed by the next VEL */
        PID_Set_Target_RPM(0.0f, 0.0f);
    }

    __set_PRIMASK(primask);
}

void UART_Comm_Send_RPM(float left_rpm, float right_rpm)
{
    char left_str[12];
    char right_str[12];
    char tx_buffer[RPM_TX_BUFFER_SIZE];

    format_float2(left_rpm,  left_str);
    format_float2(right_rpm, right_str);

    int len = snprintf(tx_buffer, sizeof(tx_buffer),
                       "RPM,%s,%s\r\n", left_str, right_str);

    if (len > 0)
    {
        if (HAL_UART_Transmit(uart, (uint8_t *)tx_buffer,
                              (uint16_t)len, TX_TIMEOUT_MS) != HAL_OK)
        {
            tx_error_count++;
        }
    }
}

void UART_Comm_Send_IMU(int16_t ax, int16_t ay, int16_t az,
                        int16_t gx, int16_t gy, int16_t gz)
{
    char tx_buffer[IMU_TX_BUFFER_SIZE];

    int len = snprintf(tx_buffer, sizeof(tx_buffer),
                       "IMU,%d,%d,%d,%d,%d,%d\r\n",
                       (int)ax, (int)ay, (int)az,
                       (int)gx, (int)gy, (int)gz);

    if (len > 0)
    {
        if (HAL_UART_Transmit(uart, (uint8_t *)tx_buffer,
                              (uint16_t)len, TX_TIMEOUT_MS) != HAL_OK)
        {
            tx_error_count++;
        }
    }
}

/* ---- HAL callbacks (run in interrupt context) ----------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != uart)
    {
        return;
    }

    if ((rx_byte == '\n') || (rx_byte == '\r'))
    {
        /* End of command */
        if (rx_index > 0)
        {
            rx_buffer[rx_index] = '\0';
            process_command(rx_buffer);
            rx_index = 0;
        }
    }
    else if (rx_index < RX_BUFFER_SIZE - 1)
    {
        rx_buffer[rx_index++] = (char)rx_byte;
    }
    else
    {
        /* Buffer overflow: discard the current command */
        rx_index = 0;
    }

    /* Keep receiving */
    HAL_UART_Receive_IT(uart, (uint8_t *)&rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == uart)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(uart, (uint8_t *)&rx_byte, 1);
    }
}
