#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "main.h"

#define MOTOR_ADDR 1

typedef enum
{
    CONTROLLER_IDLE = 0, /* 未启动 / 已停止 */
    // ZG_SETTLING,    /* 已发出运动指令，等待电机走完 + 传感器延迟 */
    CONTROLLER_ZEROING, /* 正在回零 */
    // CONTROLLER_ZERO_DONE,        /* 回零成功 */
    CONTROLLER_GET_LUT,
    CONTROLLER_BALL_STABLIZATION, // 指定位置停球
    CONTROLLER_ACC_COMP, // 加速度补偿
    CONTROLLER_STATE_COUNT
} Controller_State_t;

extern const char *controller_state_str[CONTROLLER_STATE_COUNT];

/** 中止（不会主动刹车，只是停止发新指令） */
void Controller_SetIDLE(void);
/**
 * @brief  以固定周期喂入角度（建议放在 10ms 的 MPU 采集任务里）
 *         模块内部做滑动平均，替代原来阻塞式的多次采样
 */
void Guideway_FeedAngle(float angle);
void Guideway_FeedBallPos(float pos);
void Guideway_FeedAcc(float acc);
Controller_State_t Controller_GetState(void);
void Show_State_On_OLED(uint8_t col, uint8_t row, uint8_t charSize, uint8_t colorTurn);
void Motor_Return_Origin(void);
/** 便于调试：当前滤波后角度 **/
float Guideway_GetAngle(void);

/**************** 零位校准 ************** */
/** 启动一次回零，内部会复位 PID 与所有状态 */
void ZeroGuideway_Start(void);
/**
 * @brief  轮询推进状态机，非阻塞
 *         可以任意频率调用（10ms 任务里调也行），内部自行按控制周期节流
 * @retval 当前状态
 */
Controller_State_t ZeroGuideway_Poll(void);

/******************* 构建LUT *********************** */
void Get_Guideway_LUT_Start(void);
void Get_Guideway_LUT_Poll(int pulse);

/* ************************ 平衡球 ********************** */
void BallStablization_Start(float ball_target_mm);
Controller_State_t BallStablization_Poll(void);
void BallAccComp_Start(float ball_target_mm);
void BallAccComp_Poll(void);

#endif
