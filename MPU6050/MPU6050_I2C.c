/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2021
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */

#include <math.h>
#include "MPU6050_I2C.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define FSYNC_DLPF_REG 0x1A
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define MPU_INT_EN_REG 0x38    // 中断使能寄存器
#define MPU_INTBP_CFG_REG 0x37 // 中断/旁路设置寄存器

// Setup MPU6050
#define MPU6050_ADDR 0xD0
#define MPU6050_I2C_TIMEOUT 100

// 使用六面法标定我手上的这块MPU6050，得到的偏移值
/*
标定方法：
Z 轴朝上（正常平放）Z 轴朝下（倒扣）X 轴朝上（立起来，X朝天）X 轴朝下 Y 轴朝上 Y 轴朝下
记录每次的 RAW 值（Accel_X_RAW/Y_RAW/Z_RAW）
Z 轴朝上时测得平均 raw 值为 Z_up；Z 轴朝下时测得平均 raw 值为 Z_down
则：零偏 offset_Z = (Z_up + Z_down) / 2
灵敏度对应的满量程 scale_Z = (Z_up - Z_down) / 2   // 这个值就相当于原代码里的 Accel_Z_corrector
*/
#define X_ACC_OFFSET 8448.0
#define Y_ACC_OFFSET -5301.0
#define Z_ACC_OFFSET 1987.0
#define X_ACC_SCALE 16384.0
#define Y_ACC_SCALE 16384.0
#define Z_ACC_SCALE 16585.0

uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f, // 越小，越信任"预测值"（即积分陀螺仪得到的角度），滤波结果越平滑但响应变慢
    .Q_bias = 0.003f,  // 越小，零偏估计变化越慢，收敛也越慢
    .R_measure = 0.03f // 越大，越不信任"测量值"（加速度计算出的角度），能压制加速度计噪声，但会让整体响应变慢、有轻微滞后
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, MPU6050_I2C_TIMEOUT);
    // return check; // 112
    if (check == 112) // 0x68 will be returned by the sensor if everything goes well
    {
        // // 开启data ready中断
        // Data = 0x01;
        // HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU_INT_EN_REG, 1, &Data, 1, MPU6050_I2C_TIMEOUT);
        // // data ready中断低电平触发
        // Data = 0xc0;
        // HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU_INTBP_CFG_REG, 1, &Data, 1, MPU6050_I2C_TIMEOUT);
        // 主循环调用 OLED_ShowUint(0, 0, MPU6050_Init(&hi2c1), 3, 16, 0);

        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, MPU6050_I2C_TIMEOUT);

        // FSYNC 与 DLPF配置，见数据手册11面
        Data = 0x06;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, FSYNC_DLPF_REG, 1, &Data, 1, MPU6050_I2C_TIMEOUT);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x09; // 100Hz 读取一次陀螺仪
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, MPU6050_I2C_TIMEOUT);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ±2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, MPU6050_I2C_TIMEOUT);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ±250 deg/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, MPU6050_I2C_TIMEOUT);
        return 0;
    }
    return 1;
}

/**
 * @brief  陀螺仪开机零偏校准。要求物体在校准期间保持完全静止。
 * @param  I2Cx: I2C句柄
 * @param  DataStruct: MPU6050数据结构体，校准结果会存入其中的Gyro_X/Y/Z_Offset字段
 * @param  sample_count: 采样次数，建议200~500次
 * @retval None
 */
void MPU6050_Calibrate_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct, uint16_t sample_count)
{
    uint8_t Rec_Data[6];
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint16_t valid_samples = 0;

    for (uint16_t i = 0; i < sample_count; i++)
    {
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
            I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, MPU6050_I2C_TIMEOUT);

        // 只累加读取成功的样本，避免偶发I2C错误污染校准结果
        if (status == HAL_OK)
        {
            int16_t gx_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
            int16_t gy_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
            int16_t gz_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

            sum_x += gx_raw;
            sum_y += gy_raw;
            sum_z += gz_raw;
            valid_samples++;
        }

        HAL_Delay(2); // 采样间隔，避免读取过快导致总线/传感器压力过大，可根据实际调整
    }

    // 防止全部读取失败导致除0
    if (valid_samples > 0)
    {
        DataStruct->Gyro_X_Offset = (double)sum_x / valid_samples;
        DataStruct->Gyro_Y_Offset = (double)sum_y / valid_samples;
        DataStruct->Gyro_Z_Offset = (double)sum_z / valid_samples;
    }
    else
    {
        // 全部采样失败（比如I2C总线异常），清零偏移量，退化为不做零偏修正
        DataStruct->Gyro_X_Offset = 0;
        DataStruct->Gyro_Y_Offset = 0;
        DataStruct->Gyro_Z_Offset = 0;
    }
}

HAL_StatusTypeDef MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, MPU6050_I2C_TIMEOUT);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = (DataStruct->Accel_X_RAW - X_ACC_OFFSET) / X_ACC_SCALE;
    DataStruct->Ay = (DataStruct->Accel_Y_RAW - Y_ACC_OFFSET) / Y_ACC_SCALE;
    DataStruct->Az = (DataStruct->Accel_Z_RAW - Z_ACC_OFFSET) / Z_ACC_SCALE;

    return status;
}

HAL_StatusTypeDef MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, MPU6050_I2C_TIMEOUT);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (deg/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = (DataStruct->Gyro_X_RAW - DataStruct->Gyro_X_Offset) / 131.0;
    DataStruct->Gy = (DataStruct->Gyro_Y_RAW - DataStruct->Gyro_Y_Offset) / 131.0;
    DataStruct->Gz = (DataStruct->Gyro_Z_RAW - DataStruct->Gyro_Z_Offset) / 131.0;

    return status;
}

HAL_StatusTypeDef MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, MPU6050_I2C_TIMEOUT);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

    return status;
}

HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, MPU6050_I2C_TIMEOUT);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = (DataStruct->Accel_X_RAW - X_ACC_OFFSET) / X_ACC_SCALE;
    DataStruct->Ay = (DataStruct->Accel_Y_RAW - Y_ACC_OFFSET) / Y_ACC_SCALE;
    DataStruct->Az = (DataStruct->Accel_Z_RAW - Z_ACC_OFFSET) / Z_ACC_SCALE;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = (DataStruct->Gyro_X_RAW - DataStruct->Gyro_X_Offset) / 131.0;
    DataStruct->Gy = (DataStruct->Gyro_Y_RAW - DataStruct->Gyro_Y_Offset) / 131.0;
    DataStruct->Gz = (DataStruct->Gyro_Z_RAW - DataStruct->Gyro_Z_Offset) / 131.0;

    // Kalman angle solve
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Ax * DataStruct->Ax + DataStruct->Az * DataStruct->Az);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Ay / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Ax, DataStruct->Az) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);

    return status;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};
