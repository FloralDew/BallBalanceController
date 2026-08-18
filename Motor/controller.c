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

static inline double fsign(double x)
{
    if (x > 0.0)
        return 1.0;
    if (x < 0.0)
        return -1.0;
    return 0.0;
}

static inline double fclamp(double val, double minVal, double maxVal)
{
    return fmin(fmax(val, minVal), maxVal);
}

/* ====================== 内部状态 ====================== */
static Controller_State_t s_state = CONTROLLER_IDLE;

#define FEED_ANGLE_AVG_WIN 4
static float s_fa_buf[FEED_ANGLE_AVG_WIN];
static uint8_t s_fa_buf_idx = 0;
static uint8_t s_fa_buf_cnt = 0;
static float s_angle_avg = 0.0f;
// static float s_last_out = 0.0f;

void Guideway_FeedAngle(float angle)
{
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

#define POS_AVG_WIN 4
static float s_fpos_buf[POS_AVG_WIN] = {0};
static uint8_t s_fpos_buf_idx = 0;
static uint8_t s_fpos_buf_cnt = 0;

typedef struct {
    float last_pos_avg;
    float pos_avg;
    // float velocity;
    uint32_t t_start;
    uint32_t t_last;
} BallController;

static BallController s_ballController;

void Guideway_FeedBallPos(float pos) // 单位为mm
{
    uint8_t i;
    float sum = 0.0f;

    if (pos > 300.0) // 处理坏帧，因为太接近测距仪会变成65535.0
    {
        uint8_t last_idx = (POS_AVG_WIN + s_fpos_buf_idx - 1u) % POS_AVG_WIN;
        pos = s_fpos_buf[last_idx];
    }

    s_fpos_buf[s_fpos_buf_idx] = pos;
    s_fpos_buf_idx = (uint8_t)((s_fpos_buf_idx + 1u) % POS_AVG_WIN);
    if (s_fpos_buf_cnt < POS_AVG_WIN)
        s_fpos_buf_cnt++;

    for (i = 0; i < s_fpos_buf_cnt; i++)
        sum += s_fpos_buf[i];
    s_ballController.pos_avg = sum / (float)s_fpos_buf_cnt;
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
#define ZG_DEADBAND 0.15f /* 死区，略大于 ±0.1° 的静态抖动 */
#define ZG_TARGET_ANGLE 0.0f /* 目标角度 */

typedef struct {
    PID_t pid;
    uint32_t t_start;
    uint32_t t_last; /* 上一次控制动作时刻 */
    uint8_t stable_count;
} ZeroGuidewayController;

static ZeroGuidewayController s_zgController;

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
    
    const float ZG_MAX_STEP_DEG = 2.0f; /* 单拍最大修正量（导轨度数），防止大步冲过头 */

    PID_Init(&s_zgController.pid, ZG_KP, ZG_KI, ZG_KD);
    PID_SetTarget(&s_zgController.pid, ZG_TARGET_ANGLE);
    PID_SetOutputLimit(&s_zgController.pid, -ZG_MAX_STEP_DEG, ZG_MAX_STEP_DEG);
    PID_SetIntegralLimit(&s_zgController.pid, ZG_INT_MAX, ZG_INT_SEP);
    PID_SetDeadband(&s_zgController.pid, ZG_DEADBAND);
    PID_SetDFilter(&s_zgController.pid, ZG_D_ALPHA);

    s_zgController.t_start = HAL_GetTick();
    s_zgController.t_last = s_zgController.t_start;
    s_zgController.stable_count = 0;
    // s_last_out = 0.0f;
    s_state = CONTROLLER_ZEROING;
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
    
    if (s_state != CONTROLLER_ZEROING)
        return s_state;

    uint32_t now;
    float angle, out_deg, dt;

    now = HAL_GetTick();

    /* 节流：未到控制周期就直接返回，让出 CPU */
    if ((now - s_zgController.t_last) < ZG_CTRL_PERIOD_MS)
        return s_state;

    dt = (float)(now - s_zgController.t_last) / 1000.0f;
    s_zgController.t_last = now;

    /* ---- 采样（已由 FeedAngle 滤波） ---- */
    angle = s_angle_avg;

    /* ---- 收敛判定：连续多拍落在死区内 ---- */
    if (fabsf(ZG_TARGET_ANGLE - angle) < ZG_DEADBAND)
    {
        if (++s_zgController.stable_count >= ZG_STABLE_CNT)
        {
            s_state = CONTROLLER_IDLE;
            Emm_V5_Origin_Set_O(MOTOR_ADDR, true); // 设置单圈回零零点位置，写入flash
            return s_state;
        }
    }
    else
    {
        s_zgController.stable_count = 0;
    }

    /* ---- PID 运算并执行 ---- */
    out_deg = PID_Calc(&s_zgController.pid, angle, dt); // 单位为deg
    // s_last_out = out_deg;

    uint32_t pulse;
    uint8_t dir;
    pulse = (uint32_t)(fabsf(out_deg) * 38.3f + 0.5f); // +0.5是为了四舍五入
    // 回归可得每一度大约需要38.326个脉冲

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
static int Calc_Pulse_By_Angle(double target_angle) // 这里的pulse表示顺时针旋转、向下倾斜为正
{
    const double OFFSET = 0.5867950943;

    target_angle = fclamp(target_angle, -GUIDEWAY_ANGLE_LIMIT, GUIDEWAY_ANGLE_LIMIT);
    double sqared = target_angle * target_angle;
    double cubed = sqared * target_angle;
    double pulse = ((5.867950943 - 35.58989338 * target_angle + 3.704467377 * sqared - 0.09361729684 * cubed) /
                    (1 - 0.1031174851 * target_angle + 0.002126693238 * sqared + 0.00002505123008 * cubed)) -
                   OFFSET;
    return (int)(pulse + 0.5); // 有正有负，必须和Absolute_Pos_CW_Positive配合使用
}

void BallStablization_Start(bool acc_comp)
{
    s_ballController.t_start = HAL_GetTick();
    s_ballController.t_last = s_ballController.t_start;
    s_ballController.last_pos_avg = s_ballController.pos_avg;
    s_state = CONTROLLER_BALL_STABLIZATION;
}

void BallStablization_Poll(bool acc_comp, double ball_target_cm)
{
    const double LQR_QP = 6.0; // 位置误差 e 的代价权重
    const double LQR_QV = 3.0; // 速度 v 的代价权重
    const double LQR_R = 0.3;
    const double k1 = sqrt(LQR_QP / LQR_R);
    const double k2 = sqrt(LQR_QV / LQR_R + 2.0 * sqrt(LQR_QP / LQR_R));

    const double V_STATIC_EPS = 0.003; // 判定 "接近静止" 的速度阈值(m / s)
    const double G = 9.8;              // 重力加速度
    const double MU = 0.01;            // 动摩擦因数
    if (s_state != CONTROLLER_BALL_STABLIZATION)
        return;

    uint32_t now = HAL_GetTick();
    double control_intv_s = (double)(now - s_ballController.t_last) * 0.001;
    s_ballController.t_last = now;

    double v = (s_ballController.pos_avg - s_ballController.last_pos_avg) / control_intv_s * 0.001; // 单位为m/s
    double e = s_ballController.pos_avg * 0.001 - ball_target_cm * 0.01;
    s_ballController.last_pos_avg = s_ballController.pos_avg;

    // LQR 最优反馈：期望加速度(双积分器闭式解 a = -K1 * e - K2 * v)
    double a_des = -k1 * e - k2 * v;
    // 静止/准静止时，用 "期望加速度方向" 代替 sign(v) 来判断摩擦方向 (因为速度为零时 sign(v)
    // 无定义，但摩擦仍会阻碍即将发生的运动趋势)
    double sgn;
    if (fabs(v) > V_STATIC_EPS)
        sgn = fsign(v);
    else
        sgn = fsign(a_des);

    // 反馈线性化：反解隐式方程 g * sinθ - mu * g *cosθ * sgn = a_des
    // 用不动点迭代求解(因 cosθ 依赖 θ 本身)
    double theta = asin(fclamp(a_des / G, -1.0, 1.0));
    double rhs;
    for (int _ = 0; _ < 6; _++)
    {
        rhs = (a_des + MU * G * cosf(theta) * sgn) / G;
        rhs = fclamp(rhs, -1.0, 1.0);
        theta = asin(rhs);
    }

    theta = RAD_TO_DEG * theta;
    int pulse = Calc_Pulse_By_Angle(theta); // theta是向下为正的
    Absolute_Pos_CW_Positive(100, 20, pulse);
}
