/**
 ******************************************************************************
 * @file    scheduler.h
 * @brief   裸机协作式时间片调度器（静态任务表，非抢占）
 *          任务函数必须非阻塞
 ******************************************************************************
 */
#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef void (*Task_Func_t)(void);

    typedef struct
    {
        Task_Func_t func;   /* 任务函数，必须非阻塞。这是一个函数指针 */
        uint16_t period_ms; /* 调度周期，0 表示每轮主循环都执行 */
        uint8_t enable;     /* 使能标志 */
        uint32_t last_tick; /* 内部：上次执行时刻，初值填 0 即可 */
    } Task_t;

    /**
     * @brief  绑定任务表
     * @param  list 任务数组（需为全局/静态变量）
     * @param  num  任务个数
     */
    void Sched_Init(Task_t *list, uint8_t num);

    /**
     * @brief  调度一轮，放在 while(1) 中反复调用
     */
    void Sched_Run(void);

    /**
     * @brief  动态开关某个任务（按任务表下标）
     */
    void Sched_SetEnable(uint8_t idx, uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* __SCHEDULER_H__ */
