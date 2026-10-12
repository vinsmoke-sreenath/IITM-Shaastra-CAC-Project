/*
 * mpu6050.h
 *
 * Created on: Oct 1, 2026
 * Author: sreenath_asn
 * Protocol: I2C
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include <stdint.h>

#define Device_Address          0x68
#define MPU6050_ADDR            (Device_Address << 1)
#define MPU6050_TIMEOUT         20U
#define FS_Gyro_250             0x00
#define FS_Gyro_500             0x08
#define FS_Gyro_1000            0x10
#define FS_Gyro_2000            0x18
#define FS_Acc_2g               0x00
#define FS_Acc_4g               0x08
#define FS_Acc_8g               0x10
#define FS_Acc_16g              0x18
#define Reg_Config_Gyro         0x1B
#define Reg_Config_Acc          0x1C
#define Reg_Usr_Ctrl            0x6A
#define Reg_Pwr_Mgmt_1          0x6B
#define Reg_Who_Am_I            0x75
#define Reg_Accel_Start         0x3B

typedef struct
{
    int16_t x_acc;
    int16_t y_acc;
    int16_t z_acc;

    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;

} MPU6050_Data;

HAL_StatusTypeDef mpu6050_init(void);

HAL_StatusTypeDef mpu6050_read(MPU6050_Data *data);

HAL_StatusTypeDef mpu6050_who_am_i(uint8_t *id);


#endif /* INC_MPU6050_H_ */
