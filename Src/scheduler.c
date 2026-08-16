/**
 ******************************************************************************
 * @file    scheduler.c
 ******************************************************************************
 */
#include "scheduler.h"
#include "main.h" /* HAL_GetTick */
#include <stddef.h>

static Task_t *s_task_list = NULL;
static uint8_t s_task_num = 0;

void Sched_Init(Task_t *list, uint8_t num)
{
    uint8_t i;
    uint32_t now = HAL_GetTick();

    s_task_list = list;
    s_task_num = num;

    for (i = 0; i < num; i++)
    {
        s_task_list[i].last_tick = now;
    }
}

void Sched_Run(void)
{
    uint8_t i;
    uint32_t now, elapsed;

    if (s_task_list == NULL)
        return;

    for (i = 0; i < s_task_num; i++)
    {
        Task_t *t = &s_task_list[i];

        if (!t->enable || t->func == NULL)
            continue;

        now = HAL_GetTick();

        if (t->period_ms == 0)
        {
            t->func(); /* 每轮都跑：事件型任务 */
            continue;
        }

        /* 无符号相减，天然处理 49 天溢出 */
        elapsed = now - t->last_tick;
        if (elapsed < t->period_ms)
            continue;

        /* 累加式推进，避免误差长期累积导致周期漂移；
           若延误超过两个周期，直接重同步，防止追赶式连发 */
           // 这就是不使用 last = now 的好处
        if (elapsed >= (uint32_t)t->period_ms * 2u)
            t->last_tick = now;
        else
            t->last_tick += t->period_ms;

        t->func();
    }
}

void Sched_SetEnable(uint8_t idx, uint8_t enable)
{
    if (s_task_list == NULL || idx >= s_task_num)
        return;
    s_task_list[idx].enable = enable ? 1 : 0;
}
