/*
 * mpu6050.c
 *
 * Created on: Oct 1, 2026
 * Author: sreenath_asn
 */

#include "mpu6050.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;


HAL_StatusTypeDef mpu6050_who_am_i(uint8_t *id)
{
    if (id == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        Reg_Who_Am_I,
        I2C_MEMADD_SIZE_8BIT,
        id,
        1,
        MPU6050_TIMEOUT
    );
}

HAL_StatusTypeDef mpu6050_init(void)
{
    HAL_StatusTypeDef ret;
    uint8_t data;
    uint8_t who_am_i;


    ret = HAL_I2C_IsDeviceReady(
        &hi2c1,
        MPU6050_ADDR,
        3,
        MPU6050_TIMEOUT
    );

    if (ret != HAL_OK)
    {
        return ret;
    }


    ret = mpu6050_who_am_i(&who_am_i);

    if (ret != HAL_OK)
    {
        return ret;
    }

    if (who_am_i != 0x68)
    {
        return HAL_ERROR;
    }

    data = 0x00;

    ret = HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        Reg_Pwr_Mgmt_1,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        MPU6050_TIMEOUT
    );

    if (ret != HAL_OK)
    {
        return ret;
    }
    HAL_Delay(100);


    data = FS_Gyro_500;

    ret = HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        Reg_Config_Gyro,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        MPU6050_TIMEOUT
    );

    if (ret != HAL_OK)
    {
        return ret;
    }

    data = FS_Acc_4g;

    ret = HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6050_ADDR,
        Reg_Config_Acc,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        MPU6050_TIMEOUT
    );

    if (ret != HAL_OK)
    {
        return ret;
    }

    return HAL_OK;
}


HAL_StatusTypeDef mpu6050_read(MPU6050_Data *data)
{
    HAL_StatusTypeDef ret;
    uint8_t buffer[14];

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    ret = HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6050_ADDR,
        Reg_Accel_Start,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        14,
        MPU6050_TIMEOUT
    );

    if (ret != HAL_OK)
    {
        return ret;
    }

    data->x_acc = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);

    data->y_acc = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);

    data->z_acc = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);

    data->x_gyro = (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);

    data->y_gyro = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);

    data->z_gyro = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);


    return HAL_OK;
}
