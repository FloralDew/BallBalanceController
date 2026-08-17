#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "main.h"

#define MOTOR_ADDR 1

typedef enum
{
    CONTROLLER_IDLE = 0, /* 未启动 / 已停止 */
    // ZG_SETTLING,    /* 已发出运动指令，等待电机走完 + 传感器延迟 */
    CONTROLLER_ZERO_RUNNING,     /* 正在闭环调节 */
    CONTROLLER_ZERO_DONE,        /* 回零成功 */
    CONTROLLER_STATE_COUNT
} Controller_State_t;

extern const char *state_str[CONTROLLER_STATE_COUNT];

/**
 * @brief  以固定周期喂入角度（建议放在 10ms 的 MPU 采集任务里）
 *         模块内部做滑动平均，替代原来阻塞式的多次采样
 */
void Guideway_FeedAngle(float angle);

/** 启动一次回零，内部会复位 PID 与所有状态 */
void ZeroGuideway_Start(void);

/** 中止回零（不会主动刹车，只是停止发新指令） */
void ZeroGuideway_Abort(void);

/**
 * @brief  轮询推进状态机，非阻塞
 *         可以任意频率调用（10ms 任务里调也行），内部自行按控制周期节流
 * @retval 当前状态
 */
Controller_State_t ZeroGuideway_Poll(void);

Controller_State_t Controller_GetState(void);
void Show_State_On_OLED(uint8_t col, uint8_t row, uint8_t charSize, uint8_t colorTurn);

/** 便于调试：当前滤波后角度 **/
float Guideway_GetAngle(void);

#endif
