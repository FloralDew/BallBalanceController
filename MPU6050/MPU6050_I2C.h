/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef __MPU6050_I2C_H
#define __MPU6050_I2C_H

#include "main.h"
#include <stdint.h>

#define MPU6050_GW_ADDR 0xD0
#define MPU6050_CHASSIS_ADDR 0xD2

extern const float mpu_gw_correction[6];
extern const float mpu_chassis_correction[6];

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

// MPU6050 structure
typedef struct
{
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;

    // 新增：陀螺仪零偏校准值（单位：与Gyro_X_RAW同为原始LSB单位）
    double Gyro_X_Offset;
    double Gyro_Y_Offset;
    double Gyro_Z_Offset;

    // 此校准和correction有所区别，是为了去除重力影响
    double Accel_X_Offset;
    double Accel_Y_Offset;
    double Accel_Z_Offset;

    Kalman_t KalmanX;
    Kalman_t KalmanY;

    uint32_t timer;
} MPU6050_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, uint8_t addr, MPU6050_t *DataStruct);
void MPU6050_Calibrate_Gyro(I2C_HandleTypeDef *I2Cx, uint8_t addr, MPU6050_t *DataStruct, uint16_t sample_count);
void MPU6050_Calibrate_Accel(I2C_HandleTypeDef *I2Cx, uint8_t addr, MPU6050_t *DataStruct, uint16_t sample_count,
                              const float *correction, bool channel_x, bool channel_y, bool channel_z);
HAL_StatusTypeDef MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, uint8_t addr, MPU6050_t *DataStruct, const float *correction);
HAL_StatusTypeDef MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, uint8_t addr, MPU6050_t *DataStruct);
HAL_StatusTypeDef MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, uint8_t addr, MPU6050_t *DataStruct);
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, uint8_t addr, MPU6050_t *DataStruct, const float *correction);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

#endif /* __MPU6050_I2C_H */
