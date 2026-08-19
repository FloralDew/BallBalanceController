#include "button.h"

void button_init(BTN_HandleTypedef *hbutton, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
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

void button_get(BTN_HandleTypedef *hbutton)
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

            if (hbutton->click_cnt == 1) /* 第二次按下，当场定性 */ 
            {
                hbutton->click_cnt = 0;
                if ((now - hbutton->release_tick) <= BTN_DOUBLEPRESSMS)
                    hbutton->btn_state = BTN_DOUBLEPRESS;
            }
        }
        else if (!hbutton->long_reported && (now - hbutton->press_tick) >= BTN_LONGPRESSMS)
        {
            hbutton->long_reported = 1;
            hbutton->btn_state = BTN_LONGPRESS;
        }
    }
    else
    {
        if (hbutton->down)
        {
            hbutton->down = 0;
            hbutton->release_tick = now;
            if (!hbutton->long_reported && (now - hbutton->press_tick) >= BTN_PRESSMS)
                hbutton->click_cnt = 1; /* 挂起，等窗口超时或第二次按下 */
        }
    }
}
