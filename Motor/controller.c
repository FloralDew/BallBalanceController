#include "controller.h"
#include "Emm_V5.h"
#include "pid.h"
#include <math.h>

/* ================== 硬件相关参数（必须按实际标定） ================== */
#define MOTOR_DIR_CW 0  /* 接口定义：0 = CW */
#define MOTOR_DIR_CCW 1 /* 非 0 = CCW */
#define RAD_TO_DEG 57.29577951308232

const char *controller_state_str[CONTROLLER_STATE_COUNT] =
{
    [CONTROLLER_IDLE] = "IDLE",
    [CONTROLLER_ZEROING] = "ZEROING",
    // [CONTROLLER_ZERO_DONE] = "ZERO_DONE"
    [CONTROLLER_GET_LUT] = "GETTING_LUT",
    [CONTROLLER_BALL_STABLIZATION] = "STABLIZING"
};

// static inline double fsign(double x)
// {
//     if (x > 0.0)
//         return 1.0;
//     if (x < 0.0)
//         return -1.0;
//     return 0.0;
// }

static inline double fclamp(double val, double minVal, double maxVal)
{
    return fmin(fmax(val, minVal), maxVal);
}

/* ====================== 内部状态 ====================== */
static Controller_State_t s_state = CONTROLLER_IDLE;

static float s_angle_avg = 0.0f;
void Guideway_FeedAngle(float angle)
{
    const uint8_t FEED_ANGLE_AVG_WIN = 50; // 超大窗抑制低频漂移
    static float s_fa_buf[FEED_ANGLE_AVG_WIN];
    static uint8_t s_fa_buf_idx = 0;
    static uint8_t s_fa_buf_cnt = 0;

    uint8_t i;
    float sum = 0.0f;

    s_fa_buf[s_fa_buf_idx] = angle;
    s_fa_buf_idx = (uint8_t)((s_fa_buf_idx + 1u) % FEED_ANGLE_AVG_WIN);
    if (s_fa_buf_cnt < FEED_ANGLE_AVG_WIN)
        s_fa_buf_cnt++;

    for (i = 0; i < s_fa_buf_cnt; i++)
        sum += s_fa_buf[i];
    s_angle_avg = sum / (float)s_fa_buf_cnt;
}

static float s_ball_pos = 0.0f;
void Guideway_FeedBallPos(float pos) // 单位为mm
{
    static float s_last_ball_pos = 0.0f;
    s_last_ball_pos = s_ball_pos;

    if (pos > 300.0) // 处理坏帧，因为太接近测距仪会变成65535.0
    {
        pos = s_last_ball_pos;
    }
    s_ball_pos = pos;
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

void Motor_Return_Origin(void)
{
    Emm_V5_Origin_Trigger_Return(MOTOR_ADDR, 0, false);
}

/* ***************** 控制器通用函数结束 ******************** */

/* ************************************ 导轨回零 *****************************************
 * 控制思路：
 *   PID 输出 = "本拍导轨还需要转动多少度"，再按连杆比折算成电机脉冲，
 *   用位置模式 raF = 2（相对当前电机实时位置）走一小步。
 *   由于 KalmanAngleX 有 >100ms 延迟，控制周期必须显著大于该延迟，
 *   否则会因"看到的还是旧角度"而反复过冲振荡。
 ******************************************************************************
 */
static PID_t s_zg_pid;
void ZeroGuideway_Start(void)
{
    if (s_state != CONTROLLER_IDLE)
        return;
    /* PID 参数：输出单位 = 导轨度数，误差单位 = 导轨度数。
    Kp = 1.0 相当于"一拍走完全部误差"，对有延迟的系统必然振荡，
    所以取 0.3~0.5，即每拍只吃掉一部分误差，靠多拍逼近 */
    const float ZG_KP = 0.3f;
    const float ZG_KI = 0.0f;
    const float ZG_KD = 0.03f;
    const float ZG_INT_MAX = 5.0f; /* 积分限幅 */
    const float ZG_INT_SEP = 3.0f; /* |误差|>3° 时不积分，先靠 P 快速接近 */
    const float ZG_D_ALPHA = 0.3f; /* 微分低通，噪声大就再调小 */
    const float ZG_DEADBAND = 0.15f; /* 死区，略大于 ±0.1° 的静态抖动 */

    const float ZG_MAX_STEP_DEG = 2.0f; /* 单拍最大修正量（导轨度数），防止大步冲过头 */
    const float ZG_TARGET_ANGLE = 0.0f; /* 目标角度 */

    PID_Init(&s_zg_pid, ZG_KP, ZG_KI, ZG_KD);
    PID_SetTarget(&s_zg_pid, ZG_TARGET_ANGLE);
    PID_SetOutputLimit(&s_zg_pid, -ZG_MAX_STEP_DEG, ZG_MAX_STEP_DEG);
    PID_SetIntegralLimit(&s_zg_pid, ZG_INT_MAX, ZG_INT_SEP);
    PID_SetDeadband(&s_zg_pid, ZG_DEADBAND);
    PID_SetDFilter(&s_zg_pid, ZG_D_ALPHA);

    // s_last_out = 0.0f;
    s_state = CONTROLLER_ZEROING;
}

/**
 * @brief  导轨自动回零
 * @retval ZG_OK / ZERO_ERR_TIMEOUT / ZG_ERR_SENSOR
 */
Controller_State_t ZeroGuideway_Poll(void)
{
    const float ZG_STABLE_CNT = 4;     /* 连续 N 拍在死区内才认为回零完成 */
    const int ZG_MIN_PULSE = 2;        /* 小于该脉冲数不发命令，避免无意义抖动 */
    const int ZERO_SPD_RPM = 15;
    const int ZERO_ACC = 0;

    static uint32_t s_zg_t_last; // 上次动作时刻
    static uint8_t s_init_flag = 0;

    if (!s_init_flag)
    {
        s_zg_t_last = HAL_GetTick();
        s_init_flag = 1;
    }

    static uint8_t s_zg_stable_count = 0;

    if (s_state != CONTROLLER_ZEROING)
        return s_state;

    uint32_t now;
    float angle, out_deg, dt;

    now = HAL_GetTick();

    dt = (float)(now - s_zg_t_last) / 1000.0f;
    s_zg_t_last = now;

    /* ---- 采样（已由 FeedAngle 滤波） ---- */
    angle = s_angle_avg;

    /* ---- 收敛判定：连续多拍落在死区内 ---- */
    if (fabsf(s_zg_pid.target - angle) < s_zg_pid.deadband)
    {
        if (++s_zg_stable_count >= ZG_STABLE_CNT)
        {
            s_state = CONTROLLER_IDLE;
            Emm_V5_Origin_Set_O(MOTOR_ADDR, true); // 设置单圈回零零点位置，写入flash
            return s_state;
        }
    }
    else
    {
        s_zg_stable_count = 0;
    }

    /* ---- PID 运算并执行 ---- */
    out_deg = PID_Calc(&s_zg_pid, angle, dt); // 单位为deg
    // s_last_out = out_deg;

    uint32_t pulse;
    uint8_t dir;
    pulse = (uint32_t)(fabsf(out_deg) * 36.48f + 0.5f); // +0.5是为了四舍五入
    // 回归可得每一度大约需要36.48个脉冲（线性段）

    if (pulse >= ZG_MIN_PULSE)
    {
        /* 误差为正（当前角度偏小，需要角度增大）→ 电机 CCW */
        if (out_deg > 0.0f)
            dir = MOTOR_DIR_CCW;
        else
            dir = MOTOR_DIR_CW;

        /* raF = 2：相对当前电机实时位置运动，不会累积历史目标位置误差 */
        Emm_V5_Pos_Control(MOTOR_ADDR, dir, ZERO_SPD_RPM, ZERO_ACC, pulse, 2, false);
    }
    return s_state;
}

/* *************************** 获取LUT ***************************** */

void Get_Guideway_LUT_Start(void) // 应在回零后调用
{
    s_state = CONTROLLER_GET_LUT;
}

static void Absolute_Pos_CW_Positive(uint16_t vel, uint8_t acc, int pulse)
{
    uint8_t dir = MOTOR_DIR_CW; // 当pulse > 0时向下倾斜，MPU读取angle < 0
    if (pulse < 0)
    {
        dir = MOTOR_DIR_CCW;
    }
    pulse = ABS(pulse);
    Emm_V5_Pos_Control(MOTOR_ADDR, dir, vel, acc, pulse, 1, false); // 绝对位置
}

void Get_Guideway_LUT_Poll(int pulse)
{
    const uint16_t SPD = 20;
    const uint8_t ACC = 0;
    if (s_state != CONTROLLER_GET_LUT)
        return;

    Absolute_Pos_CW_Positive(SPD, ACC, pulse);
}

/* ***************************** 钢球控制系统 ************************************* */
#define GUIDEWAY_ANGLE_LIMIT 15.0
static int Calc_Pulse_By_Angle(double target_angle) 
{
    const double OFFSET = 5.867950943;

    target_angle = fclamp(target_angle, -GUIDEWAY_ANGLE_LIMIT, GUIDEWAY_ANGLE_LIMIT);
    double sqared = target_angle * target_angle;
    double cubed = sqared * target_angle;
    double pulse = ((5.867950943 - 35.58989338 * target_angle + 3.704467377 * sqared - 0.09361729684 * cubed) /
                    (1 - 0.1031174851 * target_angle + 0.002126693238 * sqared + 0.00002505123008 * cubed)) -
                   OFFSET;
    return (int)(pulse + 0.5); // 有正有负，必须和Absolute_Pos_CW_Positive配合使用
    // 这里的pulse表示顺时针旋转、向下倾斜为正
    // theta是活动端向下为负
}

static PID_t s_ball_pid;
void BallStablization_Start(bool acc_comp)
{
    (void)acc_comp;

    const float BS_KP = 0.065f;       /* 球每偏离1 mm，导轨倾角变化的deg */
    const float BS_KI = 0.000f;       /* 误差每持续 1mm·1s，倾角累加的deg */
    const float BS_KD = 0.060f;       /* 球每1mm/s，导轨倾角变化的deg */
    const float BS_D_ALPHA = 0.25f;   /* 微分低通系数，越小越平滑，响应越慢 */
    const float BS_I_MAX = 18.0f;     /* 积分限幅 */
    const float BS_I_SEP = 20.0f;     /* |误差|>SEP 时不积分，先靠 P 快速接近 */
                                      // 补充：|误差|超过SEP时，积分项设置为0
    const float BS_POS_DB = 5.0f;     // 位置死区，单位为mm

    PID_Init(&s_ball_pid, BS_KP, BS_KI, BS_KD);
    PID_SetOutputLimit(&s_ball_pid, -GUIDEWAY_ANGLE_LIMIT, GUIDEWAY_ANGLE_LIMIT);
    PID_SetIntegralLimit(&s_ball_pid, BS_I_MAX, BS_I_SEP);
    PID_SetDFilter(&s_ball_pid, BS_D_ALPHA);
    PID_SetDeadband(&s_ball_pid, BS_POS_DB); // 这个是比例/积分作用的死区，不影响微分

    // s_bs_theta_last = 0.0f;
    // s_bs_pulse_valid = 0;
    s_state = CONTROLLER_BALL_STABLIZATION;
}

/* ************************* debug pid ***************************** */
// 必须是全局变量才能调试
float debug_kp = 0.03f;
float debug_ki = 0.05f;
float debug_kd = 0.025f;
float debug_d_alpha = 1.0f;
uint32_t debug_spd = 40;
uint8_t debug_acc = 150;
Controller_State_t BallStablization_Poll(bool acc_comp, double ball_target_cm)
{
    const uint8_t BS_STABLE_CNT = 100; // 连续100拍(2s)落在位置死区内，才认为稳定

    s_ball_pid.Kp = debug_kp;
    s_ball_pid.Ki = debug_ki;
    s_ball_pid.Kd = debug_kd;
    s_ball_pid.d_alpha = debug_d_alpha;
    /* ************** 调试结束 *************** */

    const float BS_PULSE_DB = 3;     /* ★ 死区放输出端：变化不够大就不重发指令 */
    const float BS_CTRL_DT = 0.020f; /* 调度器给的控制周期s */

    float pos, theta;
    int pulse;
    static int s_bs_pulse_last = 0;
    static uint8_t s_ball_stable_count = 0;

    (void)acc_comp;
    if (s_state != CONTROLLER_BALL_STABLIZATION)
        return s_state;

    pos = s_ball_pos;
    PID_SetTarget(&s_ball_pid, (float)ball_target_cm * 10.0f); /* cm -> mm */

    /* ---- 收敛判定：连续多拍落在死区内 ---- */
    if (fabsf(s_ball_pid.target - pos) < s_ball_pid.deadband)
    {
        if (++s_ball_stable_count >= BS_STABLE_CNT)
        {
            s_state = CONTROLLER_IDLE;
            Absolute_Pos_CW_Positive(10, 10, 0); // 缓慢回零
            return s_state;
        }
    }
    else
    {
        s_ball_stable_count = 0;
    }

    /* 输出直接就是导轨倾角(度)：库内 err = target - measure，
       d_filt = -v，展开即 -(Kp*e + Ki*∫e + Kd*v) */
    theta = PID_Calc(&s_ball_pid, pos, BS_CTRL_DT);

    if (ABS(pos - ball_target_cm * 10.0f) > s_ball_pid.int_sep)
        s_ball_pid.integral = 0.0f; // 这个逻辑是原pid库没有的，这里手动补上

    /* 输出死区：避免驱动器反复重规划梯形曲线 */
    pulse = Calc_Pulse_By_Angle((double)theta);
    if (ABS(pulse - s_bs_pulse_last) >= BS_PULSE_DB)
    {
        Absolute_Pos_CW_Positive(debug_spd, debug_acc, pulse);
        s_bs_pulse_last = pulse;
    }

    OLED_printf(0, 1, 12, 0, "%f   ", s_ball_pid.integral);
    return s_state;
}
