#include "button.h"

void Button_Init(BTN_HandleTypedef *hbutton, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    hbutton->GPIOx = GPIOx;
    hbutton->GPIO_Pin = GPIO_Pin;
    hbutton->btn_state = BTN_RELEASE;
    hbutton->click_cnt = 0;
    hbutton->down = 0;
    hbutton->press_tick = 0;
    hbutton->release_tick = 0;
    hbutton->long_reported = 0;
}

// void Button_Get(BTN_HandleTypedef *hbutton)
// {
//     uint32_t now = HAL_GetTick();

//     /* 单击：松开状态下，双击窗口内没有第二次按下 */
//     if (!hbutton->down && hbutton->click_cnt == 1 && (now - hbutton->release_tick) > BTN_DOUBLEPRESSMS)
//     {
//         hbutton->click_cnt = 0;
//         hbutton->btn_state = BTN_PRESS;
//     }

//     if (HAL_GPIO_ReadPin(hbutton->GPIOx, hbutton->GPIO_Pin) == GPIO_PIN_RESET)
//     {
//         if (!hbutton->down)
//         {
//             hbutton->down = 1;
//             hbutton->long_reported = 0;
//             hbutton->press_tick = now;

//             if (hbutton->click_cnt == 1) /* 第二次按下，当场定性 */ 
//             {
//                 hbutton->click_cnt = 0;
//                 if ((now - hbutton->release_tick) <= BTN_DOUBLEPRESSMS)
//                     hbutton->btn_state = BTN_DOUBLEPRESS;
//             }
//         }
//         else if (!hbutton->long_reported && (now - hbutton->press_tick) >= BTN_LONGPRESSMS)
//         {
//             hbutton->long_reported = 1;
//             hbutton->btn_state = BTN_LONGPRESS;
//         }
//     }
//     else
//     {
//         if (hbutton->down)
//         {
//             hbutton->down = 0;
//             hbutton->release_tick = now;
//             if (!hbutton->long_reported && (now - hbutton->press_tick) >= BTN_PRESSMS)
//                 hbutton->click_cnt = 1; /* 挂起，等窗口超时或第二次按下 */
//         }
//     }
// }

// void Button_Get(BTN_HandleTypedef *hbutton)
// {
//     uint32_t now = HAL_GetTick();
//     if (hbutton->click_cnt == 1 && (now - hbutton->release_tick) > BTN_DOUBLEPRESSMS)
//     {
//         hbutton->click_cnt = 0;
//         hbutton->btn_state = BTN_PRESS;
//     }

//     if (HAL_GPIO_ReadPin(hbutton->GPIOx, hbutton->GPIO_Pin) == GPIO_PIN_RESET)
//     {
//         if (!hbutton->down)
//         {
//             hbutton->down = 1;
//             hbutton->press_tick = now;
//         }
//         else if ((now - hbutton->press_tick) >= BTN_LONGPRESSMS && hbutton->btn_state != BTN_LONGPRESS)
//         {
//             hbutton->btn_state = BTN_LONGPRESS;
//         }
//     }
//     else
//     {
//         if (hbutton->down)
//         {
//             uint32_t delta = now - hbutton->press_tick;
//             if (delta >= BTN_PRESSMS && delta < BTN_LONGPRESSMS)
//             {
//                 if (hbutton->click_cnt == 0)
//                 {
//                     hbutton->release_tick = now;
//                     hbutton->click_cnt = 1;
//                 }
//                 else
//                 {
//                     if ((now - hbutton->release_tick) <= BTN_DOUBLEPRESSMS)
//                     {
//                         hbutton->btn_state = BTN_DOUBLEPRESS;
//                     }
//                     hbutton->click_cnt = 0;
//                 }
//             }
//             hbutton->down = 0;
//         }
//     }
// }

void Button_Get(BTN_HandleTypedef *hbutton)
{
    uint32_t now = HAL_GetTick();

    /* 单击：松开状态下，双击窗口内没有第二次按下 */
    if (!hbutton->down && hbutton->click_cnt == 1 && (now - hbutton->release_tick) > BTN_DOUBLEPRESSMS)
    {
        hbutton->click_cnt = 0;
        hbutton->btn_state = BTN_PRESS;
    }

    if (HAL_GPIO_ReadPin(hbutton->GPIOx, hbutton->GPIO_Pin) == GPIO_PIN_RESET)
    {
        if (!hbutton->down)
        {
            hbutton->down = 1;
            hbutton->long_reported = 0;
            hbutton->press_tick = now;
            /* 第二次按下只“武装”，不定性；等松开或长按超时再决定 */
            hbutton->click_cnt = (hbutton->click_cnt == 1) ? 2 : 0;
        }
        else if (!hbutton->long_reported && (now - hbutton->press_tick) >= BTN_LONGPRESSMS)
        {
            hbutton->long_reported = 1;
            hbutton->btn_state = BTN_LONGPRESS;
        }
    }
    else if (hbutton->down)
    {
        hbutton->down = 0;
        if (hbutton->long_reported)
        {
            hbutton->click_cnt = 0; /* 长按已上报，本次松开不计入点击 */
        }
        else if ((now - hbutton->press_tick) >= BTN_PRESSMS)
        {
            if (hbutton->click_cnt == 2)
            {
                hbutton->click_cnt = 0; /* 定性为双击，不再挂起单击 */
                hbutton->btn_state = BTN_DOUBLEPRESS;
            }
            else
            {
                hbutton->click_cnt = 1; /* 挂起，等窗口超时或第二次按下 */
                hbutton->release_tick = now;
            }
        }
        else
        {
            hbutton->click_cnt = 0; /* 太短，当抖动丢弃 */
        }
    }
}
