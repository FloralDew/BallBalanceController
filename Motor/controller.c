#include "controller.h"
#include "Emm_V5.h"

/* ================== 硬件相关参数（必须按实际标定） ================== */
#define DIR_CW 0  /* 接口定义：0 = CW */
#define DIR_CCW 1 /* 非 0 = CCW */

// void motor_init()
// {
//     Emm_V5_Set_QPos_Params(MOTOR_ADDR, 1, 100, 2, false); // 速度为1，加速度100，相对当前位置运动
// }

/**
 ******************************************************************************
 * 控制思路：
 *   PID 输出 = "本拍导轨还需要转动多少度"，再按连杆比折算成电机脉冲，
 *   用位置模式 raF = 2（相对当前电机实时位置）走一小步。
 *   由于 KalmanAngleX 有 >100ms 延迟，控制周期必须显著大于该延迟，
 *   否则会因"看到的还是旧角度"而反复过冲振荡。
 ******************************************************************************
 */
#include "pid.h"
#include <math.h>

/**
 * @brief  多次采样取均值，进一步压制 ±0.1° 的随机跳动
 */
static float get_guideway_angle(void)
{
    const uint8_t N = 5;
    float sum = 0.0f;
    uint8_t i;

    for (i = 0; i < N; i++)
    {
        sum += MPU6050.KalmanAngleX;
        HAL_Delay(10);
    }
    return sum / (float)N;
}

/**
 * @brief  把 PID 输出（导轨度数，带符号）转成一次电机相对运动
 * @retval 实际发出的脉冲数，0 表示本拍未动作
 */
static uint32_t zg_move(float delta_deg)
{
    const int ZG_MIN_PULSE = 3; /* 小于该脉冲数不发命令，避免无意义抖动 */

    uint32_t pulse;
    uint8_t dir;

    pulse = (uint32_t)(fabsf(delta_deg) * 100 + 0.5f); // +0.5是为了四舍五入
    // 这里100是估计的每一度需要的脉冲数，实际上是一个变数
    if (pulse < ZG_MIN_PULSE)
        return 0;

    /* 误差为正（当前角度偏小，需要角度增大）→ 电机 CCW */
    if (delta_deg > 0.0f)
        dir = DIR_CCW;
    else
        dir = DIR_CW;

    /* raF = 2：相对当前电机实时位置运动，不会累积历史目标位置误差 */
    Emm_V5_Pos_Control(MOTOR_ADDR, dir, 1, 100, pulse, 2, false); // 速度为1rpm加速度为100
    return pulse;
}

/**
 * @brief  导轨自动回零（阻塞式）
 * @retval ZG_OK / ZG_ERR_TIMEOUT / ZG_ERR_SENSOR
 */
uint8_t zero_guideway(void)
{
    const float ZG_TARGET_ANGLE = 0.0f; /* 目标角度 */
    const float ZG_DEADBAND = 0.15f;    /* 死区，略大于 ±0.1° 的静态抖动 */
    const int ZG_CTRL_PERIOD_MS = 250; /* 控制周期ms，需 > 传感器延迟 + 运动时间 */
    const float ZG_STABLE_CNT = 5;     /* 连续 N 拍在死区内才认为回零完成 */
    const float ZG_MAX_STEP_DEG = 2.0f;  /* 单拍最大修正量（导轨度数），防止大步冲过头 */
    /* PID 参数：输出单位 = 导轨度数，误差单位 = 导轨度数。
    Kp = 1.0 相当于"一拍走完全部误差"，对有延迟的系统必然振荡，
    所以取 0.3~0.5，即每拍只吃掉一部分误差，靠多拍逼近 */
    const float ZG_KP = 0.40f;
    const float ZG_KI = 0.06f;
    const float ZG_KD = 0.05f;
    const float ZG_INT_MAX = 5.0f; /* 积分限幅 */
    const float ZG_INT_SEP = 3.0f; /* |误差|>3° 时不积分，先靠 P 快速接近 */
    const float ZG_D_ALPHA = 0.3f; /* 微分低通，噪声大就再调小 */

    PID_t pid;
    uint32_t t_start, t_last, t_now;
    float angle, out, dt;
    uint8_t stable = 0;

    /* ---- PID 初始化 ---- */
    PID_Init(&pid, ZG_KP, ZG_KI, ZG_KD); // P, I, D
    PID_SetTarget(&pid, ZG_TARGET_ANGLE);
    PID_SetOutputLimit(&pid, -ZG_MAX_STEP_DEG, ZG_MAX_STEP_DEG); // 单拍最大修正角度
                                                                // 之所以是角度是因为pid的单位是近似的角度
    PID_SetIntegralLimit(&pid, ZG_INT_MAX, ZG_INT_SEP);
    PID_SetDeadband(&pid, ZG_DEADBAND); // 在死区内自动 err=0
    PID_SetDFilter(&pid, ZG_D_ALPHA);   // 微分低通，噪声大就再调小

    t_start = HAL_GetTick();
    t_last = t_start;

    while (true)
    {
        /* ---- 1. 等待：让电机走完 + 卡尔曼角度跟上（>100ms 延迟） ---- */
        HAL_Delay(ZG_CTRL_PERIOD_MS);

        t_now = HAL_GetTick();
        dt = (float)(t_now - t_last) / 1000.0f;
        t_last = t_now;
        if (dt <= 0.0f)
            dt = (float)ZG_CTRL_PERIOD_MS / 1000.0f;

        /* ---- 2. 采样 ---- */
        angle = get_guideway_angle();

        /* ---- 3. 收敛判定：连续多拍落在死区内 ---- */
        if (fabsf(ZG_TARGET_ANGLE - angle) < ZG_DEADBAND)
        {
            if (++stable >= ZG_STABLE_CNT)
                return ZG_OK;
        }
        else
        {
            stable = 0;
        }

        /* ---- 4. PID 运算并执行 ---- */
        out = PID_Calc(&pid, angle, dt);
        zg_move(out);

        // /* ---- 5. 超时保护 ---- */
        // if ((t_now - t_start) > ZG_TIMEOUT_MS)
        //     return ZG_ERR_TIMEOUT;
    }
}
