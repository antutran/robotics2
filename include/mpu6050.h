#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"

/* I2C Address */
#define MPU6050_ADDR          (0x68 << 1)
#define MPU6050_TIMEOUT       100

/* Register Map */
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_ACCEL_XOUT    0x3B
#define MPU6050_REG_GYRO_XOUT     0x43

/* Data structures */
typedef struct {
  float ax, ay, az; // g
  float gx, gy, gz; // deg/s
  float roll, pitch, yaw; // deg
} MPU6050_Data_t;

/* Public Functions */
void MPU6050_Module_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_Read_All(void);
void MPU6050_UpdateYaw(float dt);
void MPU6050_Calibrate(int samples);
void MPU6050_ResetYaw(void);

uint8_t MPU6050_IsOK(void);
const MPU6050_Data_t *MPU6050_GetData(void);

#endif
