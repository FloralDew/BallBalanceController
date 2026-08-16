/**
 ******************************************************************************
 * @file    pid.c
 * @brief   通用位置式 PID 控制器实现
 ******************************************************************************
 */
#include "pid.h"
#include <math.h>
#include <stddef.h>

void PID_Init(PID_t *p, float kp, float ki, float kd)
{
    if (p == NULL)
        return;

    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;
    p->target = 0.0f;

    p->out_max = 1.0e6f;
    p->out_min = -1.0e6f;
    p->int_max = 1.0e6f;
    p->int_sep = 0.0f;  /* 默认不启用积分分离 */
    p->deadband = 0.0f; /* 默认无死区 */
    p->d_alpha = 1.0f;  /* 默认不滤波 */

    PID_Reset(p);
}

void PID_Reset(PID_t *p)
{
    if (p == NULL)
        return;

    p->integral = 0.0f;
    p->prev_meas = 0.0f;
    p->d_filt = 0.0f;
    p->last_err = 0.0f;
    p->last_out = 0.0f;
    p->first_run = true;
}

void PID_SetTunings(PID_t *p, float kp, float ki, float kd)
{
    if (p == NULL)
        return;
    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;
}

void PID_SetTarget(PID_t *p, float target)
{
    if (p == NULL)
        return;
    p->target = target;
}

void PID_SetOutputLimit(PID_t *p, float out_min, float out_max)
{
    if (p == NULL || out_min > out_max)
        return;
    p->out_min = out_min;
    p->out_max = out_max;
}

void PID_SetIntegralLimit(PID_t *p, float int_max, float int_sep)
{
    if (p == NULL)
        return;
    p->int_max = (int_max < 0.0f) ? -int_max : int_max;
    p->int_sep = int_sep;
}

void PID_SetDeadband(PID_t *p, float deadband)
{
    if (p == NULL)
        return;
    p->deadband = (deadband < 0.0f) ? 0.0f : deadband;
}

void PID_SetDFilter(PID_t *p, float alpha)
{
    if (p == NULL)
        return;
    if (alpha <= 0.0f)
        alpha = 0.01f;
    if (alpha > 1.0f)
        alpha = 1.0f;
    p->d_alpha = alpha;
}

float PID_Calc(PID_t *p, float measure, float dt)
{
    float err, deriv, out;

    if (p == NULL || dt <= 0.0f)
        return 0.0f;

    /* ---- 1. 误差与死区 ---- */
    err = p->target - measure;
    p->last_err = err;

    if (p->deadband > 0.0f && fabsf(err) < p->deadband)
    {
        err = 0.0f; /* 落在死区内不再产生比例/积分作用 */
    }

    /* ---- 2. 积分：分离 + 限幅抗饱和 ---- */
    if (p->int_sep <= 0.0f || fabsf(err) <= p->int_sep)
    {
        p->integral += err * dt;
        if (p->integral > p->int_max)
            p->integral = p->int_max;
        if (p->integral < -p->int_max)
            p->integral = -p->int_max;
    }

    /* ---- 3. 微分先行：对测量值求导，避免目标突变时的微分冲击 ---- */
    if (p->first_run)
    {
        p->prev_meas = measure;
        p->d_filt = 0.0f;
        p->first_run = false;
    }
    deriv = -(measure - p->prev_meas) / dt;
    p->prev_meas = measure;

    /* 一阶低通，抑制传感器噪声被微分放大 */
    p->d_filt += p->d_alpha * (deriv - p->d_filt);

    /* ---- 4. 合成与限幅 ---- */
    out = p->Kp * err + p->Ki * p->integral + p->Kd * p->d_filt;

    if (out > p->out_max)
        out = p->out_max;
    if (out < p->out_min)
        out = p->out_min;

    p->last_out = out;
    return out;
}

float PID_GetError(const PID_t *p)
{
    return (p == NULL) ? 0.0f : p->last_err;
}

float PID_GetOutput(const PID_t *p)
{
    return (p == NULL) ? 0.0f : p->last_out;
}
