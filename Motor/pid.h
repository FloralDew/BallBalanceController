/**
 ******************************************************************************
 * @file    pid.h
 * @brief   通用增量可复用 PID 控制器（位置式 PID）
 *          特性：输出限幅、积分限幅、积分分离、误差死区、
 *                微分先行（对测量值求导）+ 一阶低通滤波
 ******************************************************************************
 */
#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    /* 参数 */
    float Kp;     /* 比例系数 */
    float Ki;     /* 积分系数 */
    float Kd;     /* 微分系数 */
    float target; /* 目标值 */

    /* 限制 */
    float out_max;  /* 输出上限 */
    float out_min;  /* 输出下限 */
    float int_max;  /* 积分项绝对值上限（抗饱和） */
    float int_sep;  /* 积分分离阈值：|err| > 该值时不积分；<=0 表示不启用 */
    float deadband; /* 误差死区：|err| < 该值时视为 0（抑制传感器抖动） */
    float d_alpha;  /* 微分低通系数 0~1，越小滤波越强，1 表示不滤波 */

    /* 内部状态 */
    float integral;  /* 积分累加值 */
    float prev_meas; /* 上一次测量值 */
    float d_filt;    /* 滤波后的微分值 */
    float last_err;  /* 上一次误差（供外部查询） */
    float last_out;  /* 上一次输出（供外部查询） */
    bool first_run;  /* 首次运行标志 */
} PID_t;

/* 初始化：内部会调用 PID_Reset，并给出宽松的默认限幅 */
void PID_Init(PID_t *p, float kp, float ki, float kd);

/* 清除积分、微分等历史状态（每次开始一段新的调节前调用） */
void PID_Reset(PID_t *p);

void PID_SetTunings(PID_t *p, float kp, float ki, float kd);
void PID_SetTarget(PID_t *p, float target);
void PID_SetOutputLimit(PID_t *p, float out_min, float out_max);
void PID_SetIntegralLimit(PID_t *p, float int_max, float int_sep);
void PID_SetDeadband(PID_t *p, float deadband);
void PID_SetDFilter(PID_t *p, float alpha);

/**
 * @brief  执行一次 PID 运算
 * @param  p       PID 句柄
 * @param  measure 当前测量值
 * @param  dt      距上次调用的时间间隔（单位：秒，必须 > 0）
 * @retval 控制量（已限幅）
 */
float PID_Calc(PID_t *p, float measure, float dt);

float PID_GetError(const PID_t *p);
float PID_GetOutput(const PID_t *p);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
