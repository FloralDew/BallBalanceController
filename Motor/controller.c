#include "controller.h"
#include "Emm_V5.h"

/* ================== 硬件相关参数（必须按实际标定） ================== */
#define DIR_CW 0  /* 接口定义：0 = CW */
#define DIR_CCW 1 /* 非 0 = CCW */

const char *controller_state_str[CONTROLLER_STATE_COUNT] =
{
    [CONTROLLER_IDLE] = "IDLE",
    [CONTROLLER_ZERO_RUNNING] = "ZERO_RUNNING",
    // [CONTROLLER_ZERO_DONE] = "ZERO_DONE"
    [CONTROLLER_GET_LUT] = "GET_LUT"
};

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

/* ====================== 内部状态 ====================== */
static PID_t s_pid;
static Controller_State_t s_state = CONTROLLER_IDLE;
static uint32_t s_t_start = 0;
static uint32_t s_t_last = 0; /* 上一次控制动作时刻 */
static uint8_t s_stable_cnt = 0;

#define ZG_AVG_WIN 4
static float s_buf[ZG_AVG_WIN];
static uint8_t s_buf_idx = 0;
static uint8_t s_buf_cnt = 0;
static float s_angle_avg = 0.0f;
// static float s_last_out = 0.0f;

void Guideway_FeedAngle(float angle)
{
    uint8_t i;
    float sum = 0.0f;

    s_buf[s_buf_idx] = angle;
    s_buf_idx = (uint8_t)((s_buf_idx + 1u) % ZG_AVG_WIN);
    if (s_buf_cnt < ZG_AVG_WIN)
        s_buf_cnt++;

    for (i = 0; i < s_buf_cnt; i++)
        sum += s_buf[i];
    s_angle_avg = sum / (float)s_buf_cnt;
}

void Controller_Abort(void)
{
    s_state = CONTROLLER_IDLE;
}

Controller_State_t Controller_GetState(void) { return s_state; }
void Show_State_On_OLED(uint8_t col, uint8_t row, uint8_t charSize, uint8_t colorTurn)
{
    OLED_printf(col, row, charSize, colorTurn, controller_state_str[s_state]);
}

float Guideway_GetAngle(void) { return s_angle_avg; }

/* ***************** 控制器通用函数结束 ******************** */

/* ******************* 导轨回零 ******************* */
#define ZG_DEADBAND 0.15f /* 死区，略大于 ±0.1° 的静态抖动 */
#define ZG_TARGET_ANGLE 0.0f /* 目标角度 */

void ZeroGuideway_Start(void)
{
    if (s_state != CONTROLLER_IDLE)
        return;
    /* PID 参数：输出单位 = 导轨度数，误差单位 = 导轨度数。
    Kp = 1.0 相当于"一拍走完全部误差"，对有延迟的系统必然振荡，
    所以取 0.3~0.5，即每拍只吃掉一部分误差，靠多拍逼近 */
    const float ZG_KP = 0.2f;
    const float ZG_KI = 0.0f;
    const float ZG_KD = 0.03f;
    const float ZG_INT_MAX = 5.0f; /* 积分限幅 */
    const float ZG_INT_SEP = 3.0f; /* |误差|>3° 时不积分，先靠 P 快速接近 */
    const float ZG_D_ALPHA = 0.3f; /* 微分低通，噪声大就再调小 */
    
    const float ZG_MAX_STEP_DEG = 2.0f; /* 单拍最大修正量（导轨度数），防止大步冲过头 */

    PID_Init(&s_pid, ZG_KP, ZG_KI, ZG_KD);
    PID_SetTarget(&s_pid, ZG_TARGET_ANGLE);
    PID_SetOutputLimit(&s_pid, -ZG_MAX_STEP_DEG, ZG_MAX_STEP_DEG);
    PID_SetIntegralLimit(&s_pid, ZG_INT_MAX, ZG_INT_SEP);
    PID_SetDeadband(&s_pid, ZG_DEADBAND);
    PID_SetDFilter(&s_pid, ZG_D_ALPHA);

    s_t_start = HAL_GetTick();
    s_t_last = s_t_start;
    s_stable_cnt = 0;
    // s_last_out = 0.0f;
    s_state = CONTROLLER_ZERO_RUNNING;
}

/**
 * @brief  导轨自动回零
 * @retval ZG_OK / ZERO_ERR_TIMEOUT / ZG_ERR_SENSOR
 */
Controller_State_t ZeroGuideway_Poll(void)
{
    const int ZG_CTRL_PERIOD_MS = 250; /* 控制周期ms，需 > 传感器延迟 + 运动时间 */
    const float ZG_STABLE_CNT = 4;     /* 连续 N 拍在死区内才认为回零完成 */
    const int ZG_MIN_PULSE = 2;        /* 小于该脉冲数不发命令，避免无意义抖动 */
    const int ZERO_SPD_RPM = 15;
    const int ZERO_ACC = 0;

    uint32_t now;
    float angle, out_deg, dt;

    if (s_state != CONTROLLER_ZERO_RUNNING)
        return s_state;

    now = HAL_GetTick();

    /* 节流：未到控制周期就直接返回，让出 CPU */
    if ((now - s_t_last) < ZG_CTRL_PERIOD_MS)
        return s_state;

    dt = (float)(now - s_t_last) / 1000.0f;
    s_t_last = now;

    /* ---- 采样（已由 FeedAngle 滤波） ---- */
    angle = s_angle_avg;

    /* ---- 收敛判定：连续多拍落在死区内 ---- */
    if (fabsf(ZG_TARGET_ANGLE - angle) < ZG_DEADBAND)
    {
        if (++s_stable_cnt >= ZG_STABLE_CNT)
        {
            s_state = CONTROLLER_IDLE;
            return s_state;
        }
    }
    else
    {
        s_stable_cnt = 0;
    }

    /* ---- PID 运算并执行 ---- */
    out_deg = PID_Calc(&s_pid, angle, dt); // 单位为deg
    // s_last_out = out_deg;

    uint32_t pulse;
    uint8_t dir;
    pulse = (uint32_t)(fabsf(out_deg) * 100 + 0.5f); // +0.5是为了四舍五入
    // 这里100是估计的每一度需要的脉冲数，实际上是一个变数

    if (pulse >= ZG_MIN_PULSE)
    {
        /* 误差为正（当前角度偏小，需要角度增大）→ 电机 CCW */
        if (out_deg > 0.0f)
            dir = DIR_CCW;
        else
            dir = DIR_CW;

        /* raF = 2：相对当前电机实时位置运动，不会累积历史目标位置误差 */
        Emm_V5_Pos_Control(MOTOR_ADDR, dir, ZERO_SPD_RPM, ZERO_ACC, pulse, 2, false);
    }
    return s_state;
}

/* ******************************* 获取LUT ************************************ */

void Get_Guideway_LUT_Start(void)
{
    s_state = 
}

Controller_State_t Get_Guideway_LUT_Poll(void)
{

}




