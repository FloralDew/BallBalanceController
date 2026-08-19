#include "main.h"

#define BTN_LONGPRESSMS 1000
#define BTN_PRESSMS 10
#define BTN_DOUBLEPRESSMS 300

// key control
typedef enum
{
    BTN_RELEASE = 0,
    BTN_PRESS,
    BTN_LONGPRESS,
    BTN_DOUBLEPRESS
} BTN_StateTypedef;
typedef struct
{
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
    volatile uint8_t  down;
    volatile uint8_t  click_cnt;
    volatile uint8_t  long_reported;   // 新增：本次按下已上报过长按
    volatile uint32_t press_tick;
    volatile uint32_t release_tick;
    BTN_StateTypedef btn_state;
} BTN_HandleTypedef;

void button_init(BTN_HandleTypedef *hbutton, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void button_get(BTN_HandleTypedef *hbutton);
