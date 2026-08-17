#ifndef __OLED_H
#define __OLED_H

#include "main.h"

// void OLED_WR_CMD(uint8_t cmd);
// void OLED_WR_DATA(uint8_t data);
void OLED_Init(uint8_t contrast);
void OLED_Clear(uint8_t start, uint8_t end);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Set_Pos(uint8_t x, uint8_t row);
// void OLED_On(void);
void OLED_ShowUint(uint8_t x, uint8_t row, unsigned int num, uint8_t len, uint8_t size2, uint8_t colorTurn);
void OLED_ShowFloat(uint8_t x, uint8_t row, float num, uint8_t z_len, uint8_t f_len, uint8_t size2, uint8_t colorTurn);
void OLED_ShowChar(uint8_t x, uint8_t row, uint8_t chr, uint8_t charSize, uint8_t colorTurn);
void OLED_ShowString(uint8_t x, uint8_t row, char *chr, uint8_t charSize, uint8_t colorTurn);
void OLED_printf(uint8_t col, uint8_t row, uint8_t charSize, uint8_t colorTurn, const char *fmt, ...);
void OLED_ShowChinese(uint8_t x, uint8_t row, uint8_t no, uint8_t colorTurn);
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *BMP, uint8_t colorTurn);
// void OLED_HorizontalShift(uint8_t direction);
// void OLED_Some_HorizontalShift(uint8_t direction, uint8_t start, uint8_t end);
// void OLED_VerticalAndHorizontalShift(uint8_t direction);
void OLED_DisplayMode(uint8_t mode);
void OLED_IntensityControl(uint8_t intensity);

#endif
