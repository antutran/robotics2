#include "mpu6050.h"
#include <stdio.h>
#include <math.h>

static I2C_HandleTypeDef *mpu_i2c = NULL;
static uint8_t            mpu_ok  = 0;
static MPU6050_Data_t     mpu_data = {0};

/* Calib data */
static float gz_bias = 0.0f;
static float gy_bias = 0.0f;
static float gx_bias = 0.0f;

void MPU6050_Module_Init(I2C_HandleTypeDef *hi2c) {
    mpu_i2c = hi2c;
}

uint8_t MPU6050_IsOK(void) { return mpu_ok; }
const MPU6050_Data_t *MPU6050_GetData(void) { return &mpu_data; }

HAL_StatusTypeDef MPU6050_Init(void) {
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(mpu_i2c, MPU6050_ADDR, 3, 50);
    if (ret != HAL_OK) { mpu_ok = 0; return ret; }

    uint8_t wakeup = 0x00;
    HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &wakeup, 1, MPU6050_TIMEOUT);
    
    mpu_ok = 1;
    MPU6050_Calibrate(500); // Calibrate bias at start
    return HAL_OK;
}

void MPU6050_Calibrate(int samples) {
    if (!mpu_ok) return;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;
    uint8_t buf[6];

    printf("[MPU6050] Calibrating... Keep robot still!\r\n");
    for (int i = 0; i < samples; i++) {
        HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR, MPU6050_REG_GYRO_XOUT, I2C_MEMADD_SIZE_8BIT, buf, 6, MPU6050_TIMEOUT);
        gx_sum += (int16_t)((buf[0] << 8) | buf[1]);
        gy_sum += (int16_t)((buf[2] << 8) | buf[3]);
        gz_sum += (int16_t)((buf[4] << 8) | buf[5]);
        HAL_Delay(2);
    }
    gx_bias = (float)gx_sum / samples / 131.0f;
    gy_bias = (float)gy_sum / samples / 131.0f;
    gz_bias = (float)gz_sum / samples / 131.0f;
    printf("[MPU6050] Bias: GZ=%.4f\r\n", gz_bias);
}

void MPU6050_ResetYaw(void) {
    mpu_data.yaw = 0.0f;
}

HAL_StatusTypeDef MPU6050_Read_All(void) {
    if (!mpu_ok) return HAL_ERROR;
    uint8_t buf[14];
    HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT, I2C_MEMADD_SIZE_8BIT, buf, 14, MPU6050_TIMEOUT);

    int16_t rax = (int16_t)((buf[0]<<8)|buf[1]);
    int16_t ray = (int16_t)((buf[2]<<8)|buf[3]);
    int16_t raz = (int16_t)((buf[4]<<8)|buf[5]);
    int16_t rgx = (int16_t)((buf[8]<<8)|buf[9]);
    int16_t rgy = (int16_t)((buf[10]<<8)|buf[11]);
    int16_t rgz = (int16_t)((buf[12]<<8)|buf[13]);

    mpu_data.ax = rax / 16384.0f;
    mpu_data.ay = ray / 16384.0f;
    mpu_data.az = raz / 16384.0f;
    mpu_data.gx = (rgx / 131.0f) - gx_bias;
    mpu_data.gy = (rgy / 131.0f) - gy_bias;
    mpu_data.gz = (rgz / 131.0f) - gz_bias;

    // Simple Roll/Pitch from Accel
    mpu_data.roll  = atan2f(mpu_data.ay, mpu_data.az) * 180.0f / M_PI;
    mpu_data.pitch = atan2f(-mpu_data.ax, sqrtf(mpu_data.ay*mpu_data.ay + mpu_data.az*mpu_data.az)) * 180.0f / M_PI;

    return HAL_OK;
}

void MPU6050_UpdateYaw(float dt) {
    if (!mpu_ok) return;
    // Basic integration of z-gyro
    // dt in seconds
    mpu_data.yaw += mpu_data.gz * dt;
}
