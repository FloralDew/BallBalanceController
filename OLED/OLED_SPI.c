#include "OLED_SPI.h"
#include "codetab.h" // 必须直接包含这个头文件，如果在oled_i2c.h中包含，此头文件中的变量无法找到
#include <stdarg.h>
#include <stdio.h>

// 注意这里直接使用了hi2c1句柄, 如果显示屏接的是别的i2c口, 需要改. 见h文件中的extern

#define COLUMN_ADDRESS_SHIFT (uint8_t)2
/*
SH1106 芯片内部的显示 RAM（GDDRAM）宽度是132 列0-131），
但绝大多数 0.96 寸 128×64 OLED模组的玻璃面板实际只用中间的 128 列去接驱动 IC 的输出端
不同厂家/批次的模组，把这 128 列接在 GDDRAM 的哪个位置是不一样的，常见的有两种：
- 从第 0 列开始接（列偏移 0）
- 从第 2 列开始接（列偏移 2），即实际可视像素对应 RAM 的第 2~129 列
*/

#define SH_Command() HAL_GPIO_WritePin(OLED_SPI_DC_GPIO_Port, OLED_SPI_DC_Pin, GPIO_PIN_RESET)
#define SH_Data() HAL_GPIO_WritePin(OLED_SPI_DC_GPIO_Port, OLED_SPI_DC_Pin, GPIO_PIN_SET)
#define SH_ResHi() HAL_GPIO_WritePin(OLED_SPI_RES_GPIO_Port, OLED_SPI_RES_Pin, GPIO_PIN_SET)
#define SH_ResLo() HAL_GPIO_WritePin(OLED_SPI_RES_GPIO_Port, OLED_SPI_RES_Pin, GPIO_PIN_RESET)
// #define SH_CsHi() HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_SET)
// #define SH_CsLo() HAL_GPIO_WritePin(GPIOA, CS_Pin, GPIO_PIN_RESET) // 只有一个显示屏，不需要片选

#define SPI_TRANSMIT_TIMEOUT_MS 2

static void OLED_WR_CMD(uint8_t cmd) // 写命令
{
	SH_Command();
	// SH_CsLo();
	HAL_SPI_Transmit(&hspi2, &cmd, 1, SPI_TRANSMIT_TIMEOUT_MS); // 阻塞
	// SH_CsHi();
}

static void OLED_WR_DATA(uint8_t data) // 写数据
{
	SH_Data();
	HAL_SPI_Transmit(&hspi2, &data, 1, SPI_TRANSMIT_TIMEOUT_MS);
}

static void OLED_WR_DATA_Multi(uint8_t *data, uint16_t len) // 一次写多个数据, 清零专用
{
	SH_Data();
	HAL_SPI_Transmit(&hspi2, data, len, SPI_TRANSMIT_TIMEOUT_MS);
}

void OLED_Init(uint8_t contrast)
{
	SH_ResLo();
	HAL_Delay(100);
	SH_ResHi();
	HAL_Delay(100);

	OLED_WR_CMD(0xAE); // display off
	OLED_WR_CMD(0x20); // Set Memory Addressing Mode
	OLED_WR_CMD(0x10); // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	OLED_WR_CMD(0xb0); // Set Page Start Address for Page Addressing Mode,0-7
	OLED_WR_CMD(0xc8); // Set COM Output Scan Direction
	OLED_WR_CMD(0x00 + COLUMN_ADDRESS_SHIFT); //---set low column address
	OLED_WR_CMD(0x10); //---set high column address
	OLED_WR_CMD(0x40); //--set start line address
	OLED_WR_CMD(0x81); //--set contrast control register
	OLED_WR_CMD(contrast); // 亮度调节 0x00-0xff
	OLED_WR_CMD(0xa1); //--set segment re-map 0 to 127
	OLED_WR_CMD(0xa6); //--set normal display
	OLED_WR_CMD(0xa8); //--set multiplex ratio(1 to 64)
	OLED_WR_CMD(0x3F); //
	OLED_WR_CMD(0xa4); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	OLED_WR_CMD(0xd3); //-set display offset
	OLED_WR_CMD(0x00); //-not offset
	OLED_WR_CMD(0xd5); //--set display clock divide ratio/oscillator frequency
	OLED_WR_CMD(0xf0); //--set divide ratio
	OLED_WR_CMD(0xd9); //--set pre-charge period
	OLED_WR_CMD(0x22); //
	OLED_WR_CMD(0xda); //--set com pins hardware configuration
	OLED_WR_CMD(0x12);
	OLED_WR_CMD(0xdb); //--set vcomh
	OLED_WR_CMD(0x20); // 0x20,0.77xVcc
	OLED_WR_CMD(0x8d); //--set DC-DC enable
	OLED_WR_CMD(0x14); //
	OLED_WR_CMD(0xaf); //--turn on oled panel
}

/**
 * @function: void OLED_On(void)
 * @description: 更新显示

 * @return {*}
 */
// void OLED_On(void)
// {
// 	uint8_t i, n;
// 	for (i = 0; i < 8; i++)
// 	{
// 		OLED_WR_CMD(0xb0 + i); // 设置页地址（0-7）
// 		OLED_WR_CMD(0x00 + COLUMN_ADDRESS_SHIFT); // 设置显示位置—列低地址
// 		OLED_WR_CMD(0x10);	   // 设置显示位置—列高地址
// 		for (n = 0; n < 128; n++)
// 			OLED_WR_DATA(1);
// 	}
// }

/**
 * @function: void OLED_Set_Pos(uint8_t x, uint8_t row)
 * @description: 坐标设置
 * @param {uint8_t} x,row
 * @return {*}
 */
void OLED_Set_Pos(uint8_t x, uint8_t row)
{
	x += COLUMN_ADDRESS_SHIFT;
	OLED_WR_CMD(0xb0 + row);				   // 设置页地址（0-7）
	OLED_WR_CMD(((x & 0xf0) >> 4) | 0x10); // 设置显示位置—列高地址
	OLED_WR_CMD((x) & 0x0f);			   // 设置显示位置—列低地址
}

/**
 * @brief 对page [start, end]清屏(包含端点)
 * @param {uint8_t} 0 <= start <= end <= 7
 * @return {*}
 */
void OLED_Clear(uint8_t start, uint8_t end)
{
	if (end < start || end > 7)
		return;
	uint8_t zero[128] = {0};
	for (uint8_t i = start; i <= end; i++)
	{
		OLED_Set_Pos(0, i);
		OLED_WR_DATA_Multi(zero, 128); // 一次性发128字节，而不是循环128次
	}
}

/**
 * @function: void OLED_Display_On(void)
 * @description: 开启OLED显示
 * @return {*}
 */
void OLED_Display_On(void)
{
	OLED_WR_CMD(0X8D); // SET DCDC命令 (电荷泵使能)
	OLED_WR_CMD(0X14); // DCDC ON (开启电荷泵)
	OLED_WR_CMD(0XAF); // DISPLAY ON,打开显示
}

/**
 * @function: void OLED_Display_Off(void)
 * @description: 关闭OLED显示
 * @return {*}
 */
void OLED_Display_Off(void)
{
	OLED_WR_CMD(0X8D); // SET DCDC命令
	OLED_WR_CMD(0X10); // DCDC OFF
	OLED_WR_CMD(0XAE); // DISPLAY OFF，关闭显示
}

/**
 * @function: unsigned int oled_pow(uint8_t m,uint8_t n)
 * @description: m^n函数
 * @param {uint8_t} m,n
 * @return {unsigned int} result
 */
unsigned int oled_pow(uint8_t m, uint8_t n)
{
	unsigned int result = 1;
	while (n--)
		result *= m;
	return result;
}

/**
 * @function: void OLED_ShowChar(uint8_t x, uint8_t row, uint8_t chr, uint8_t charSize,uint8_t colorTurn)
 * @description: 在OLED12864特定位置开始显示一个字符
 * @param {uint8_t} x字符开始显示的横坐标
 * @param {uint8_t} y字符开始显示的纵坐标
 * @param {uint8_t} chr待显示的字符
 * @param {uint8_t} charSize待显示字符的字体大小,选择字体 16/12
 * @param {uint8_t} colorTurn是否反相显示(1反相、0不反相)
 * @return {*}
 */
void OLED_ShowChar(uint8_t x, uint8_t row, uint8_t chr, uint8_t charSize, uint8_t colorTurn)
{
	unsigned char c = 0, i = 0;
	c = chr - ' '; // 得到偏移后的值
	if (x > 128 - 1)
	{
		x = 0;
		if (charSize == 12)
			row++;
		else
			row += 2;
	}
	if (charSize == 16)
	{
		OLED_Set_Pos(x, row);
		for (i = 0; i < 8; i++)
		{
			if (colorTurn)
				OLED_WR_DATA(~F8X16[c * 16 + i]);
			else
				OLED_WR_DATA(F8X16[c * 16 + i]);
		}
		OLED_Set_Pos(x, row + 1);
		for (i = 0; i < 8; i++)
		{
			if (colorTurn)
				OLED_WR_DATA(~F8X16[c * 16 + i + 8]);
			else
				OLED_WR_DATA(F8X16[c * 16 + i + 8]);
		}
	}
	else
	{
		OLED_Set_Pos(x, row);
		for (i = 0; i < 6; i++)
		{
			if (colorTurn)
				OLED_WR_DATA(~F6x8[c][i]);
			else
				OLED_WR_DATA(F6x8[c][i]);
		}
	}
}

/**
 * @function: void OLED_ShowString(uint8_t x, uint8_t row, uint8_t *chr, uint8_tcharSize, uint8_t colorTurn)
 * @description: 在OLED12864特定位置开始显示字符串
 * @param {uint8_t} x待显示字符串的开始横坐标 x:0-127
 * @param {uint8_t} y待显示字符串的开始纵坐标 row:0-7，若选择字体大小为16，则两行数字之间需要间隔2，若选择字体大小为12，间隔1
 * @param {uint8_t*} chr待显示的字符串
 * @param {uint8_t} charSize待显示字符串的字体大小,选择字体 16/12，16为8X16，12为6x8
 * @param {uint8_t} colorTurn是否反相显示(1反相、0不反相)
 * @return {*}
 */
void OLED_ShowString(uint8_t x, uint8_t row, char *chr, uint8_t charSize, uint8_t colorTurn)
{
	uint8_t j = 0;
	while (chr[j] != '\0')
	{
		OLED_ShowChar(x, row, chr[j], charSize, colorTurn);
		if (charSize == 12) // 6X8的字体列加6，显示下一个字符
			x += 6;
		else // 8X16的字体列加8，显示下一个字符
			x += 8;

		if (x > 122 && charSize == 12) // TextSize6x8如果一行不够显示了，从下一行继续显示
		{
			x = 0;
			row++;
		}
		if (x > 120 && charSize == 16) // TextSize8x16如果一行不够显示了，从下一行继续显示
		{
			x = 0;
			row += 2;
		}
		j++;
	}
}

/**
  * @brief          formatted output in oled 128*64
  * @param[in]      row: row of character string begin, 0 <= row <= 4;
  * @param[in]      col: column of character string begin, 0 <= col <= 20;
  * @param          *fmt: the pointer to format character string
  * @note           if the character length is more than one row at a time, the extra characters will be truncated
  * @retval         none
  */
/**
  * @brief          格式输出
  * @param[in]      row: 开始列，0 <= row <= 4;
  * @param[in]      col: 开始行， 0 <= col <= 20;
  * @param[in]      *fmt:格式化输出字符串
  * @note           如果字符串长度大于一行，额外的字符会换行
  * @retval         none
  */
void OLED_printf(uint8_t col, uint8_t row, uint8_t charSize, uint8_t colorTurn, const char *fmt, ...)
{
    char STRING_BUF[256] = {0};
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(STRING_BUF, sizeof(STRING_BUF), fmt, ap);
    va_end(ap);

	uint8_t x;
	if (charSize == 12)
		x = col * 6;
	else
		x = col * 8;
	OLED_ShowString(x, row, STRING_BUF, charSize, colorTurn);
}

/**
 * @function: void OLED_ShowUint(uint8_t x,uint8_t row,unsigned int num,uint8_t len,uint8_t size2, colorTurn)
 * @description: 显示数字
 * @param {uint8_t} x待显示的数字起始横坐标,x:0-126
 * @param {uint8_t} y待显示的数字起始纵坐标, row:0-7，若选择字体大小为16，则两行数字之间需要间隔2，若选择字体大小为12，间隔1
 * @param {unsigned int} num:输入的数据
 * @param {uint8_t } len:输入的数据位数
 * @param {uint8_t} size2:输入的数据大小，选择 16/12，16为8X16，12为6x8
 * @param {uint8_t} colorTurn是否反相显示(1反相、0不反相)
 * @return {*}
 */
void OLED_ShowUint(uint8_t x, uint8_t row, unsigned int num, uint8_t len, uint8_t size2, uint8_t colorTurn)
{
	uint8_t t, temp;
	uint8_t enshow = 0;
	for (t = 0; t < len; t++)
	{
		temp = (num / oled_pow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < (len - 1))
		{
			if (temp == 0)
			{
				OLED_ShowChar(x + (size2 / 2) * t, row, ' ', size2, colorTurn);
				continue;
			}
			else
				enshow = 1;
		}
		OLED_ShowChar(x + (size2 / 2) * t, row, temp + '0', size2, colorTurn);
	}
}

/**
 * @function: void OLED_ShowFloat(uint8_t x,uint8_t row,float num,uint8_t z_len,uint8_t f_len,uint8_t size2, uint8_t colorTurn)
 * @description: 显示正负浮点数
 * @param {uint8_t} x待显示的数字起始横坐标,x:0-126
 * @param {uint8_t} y待显示的数字起始纵坐标, row:0-7，若选择字体大小为16，则两行数字之间需要间隔2，若选择字体大小为12，间隔1
 * @param {float} num:输入的浮点型数据
 * @param {uint8_t } z_len:整数部分的位数
 * @param {uint8_t } f_len: 小数部分的位数
 * @param {uint8_t} size2:输入的数据大小，选择 16/12，16为8X16，12为6x8
 * @param {uint8_t} colorTurn是否反相显示(1反相、0不反相)
 * @return {*}
 */
void OLED_ShowFloat(uint8_t x, uint8_t row, float num, uint8_t z_len, uint8_t f_len, uint8_t size2, uint8_t colorTurn)
{
	uint8_t t, temp, i = 0; // i为负数标志位
	uint8_t enshow;
	int z_temp, f_temp;
	if (num < 0)
	{
		z_len += 1;
		i = 1;
		num = -num;
	}
	z_temp = (int)num;
	// 整数部分
	for (t = 0; t < z_len; t++)
	{
		temp = (z_temp / oled_pow(10, z_len - t - 1)) % 10;
		if (enshow == 0 && t < (z_len - 1))
		{
			if (temp == 0)
			{
				OLED_ShowChar(x + (size2 / 2) * t, row, ' ', size2, colorTurn);
				continue;
			}
			else
				enshow = 1;
		}
		OLED_ShowChar(x + (size2 / 2) * t, row, temp + '0', size2, colorTurn);
	}
	// 小数点
	OLED_ShowChar(x + (size2 / 2) * (z_len), row, '.', size2, colorTurn);

	f_temp = (int)((num - z_temp) * (oled_pow(10, f_len)));
	// 小数部分
	for (t = 0; t < f_len; t++)
	{
		temp = (f_temp / oled_pow(10, f_len - t - 1)) % 10;
		OLED_ShowChar(x + (size2 / 2) * (t + z_len) + 5, row, temp + '0', size2, colorTurn);
	}
	if (i == 1) // 如果为负，就将最前的一位赋值‘-’
	{
		OLED_ShowChar(x, row, '-', size2, colorTurn);
		i = 0;
	}
}

/**
 * @function: void OLED_ShowChinese(uint8_t x,uint8_t row,uint8_t no, uint8_t colorTurn)
 * @description: 在OLED特定位置开始显示16X16汉字
 * @param {uint8_t} x待显示的汉字起始横坐标x: 0-112，两列汉字之间需要间隔16
 * @param {uint8_t} y待显示的汉字起始纵坐标 row: 0-6 , 两行汉字之间需要间隔2
 * @param {uint8_t} no待显示的汉字编号
 * @param {uint8_t} colorTurn是否反相显示(1反相、0不反相)
 * @return {*}
 */
void OLED_ShowChinese(uint8_t x, uint8_t row, uint8_t no, uint8_t colorTurn)
{
	uint8_t t = 0;
	OLED_Set_Pos(x, row);
	for (t = 0; t < 16; t++)
	{
		if (colorTurn)
			OLED_WR_DATA(~Hzk[2 * no][t]); // 显示汉字的上半部分
		else
			OLED_WR_DATA(Hzk[2 * no][t]); // 显示汉字的上半部分
	}

	OLED_Set_Pos(x, row + 1);
	for (t = 0; t < 16; t++)
	{
		if (colorTurn)
			OLED_WR_DATA(~Hzk[2 * no + 1][t]); // 显示汉字的上半部分
		else
			OLED_WR_DATA(Hzk[2 * no + 1][t]); // 显示汉字的上半部分
	}
}

/**
 * @function: void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *  BMP,uint8_t colorTurn)
 * @description: 在OLED特定区域显示BMP图片
 * @param {uint8_t} x0图像开始显示横坐标  x0:0-127
 * @param {uint8_t} y0图像开始显示纵坐标  y0:0-7
 * @param {uint8_t} x1图像结束显示横坐标  x1:1-128
 * @param {uint8_t} y1图像结束显示纵坐标  y1:1-8
 * @param {uint8_t} *BMP待显示的图像数据
 * @param {uint8_t} colorTurn是否反相显示(1反相、0不反相)
 * @return {*}
 */
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *BMP, uint8_t colorTurn)
{
	uint32_t j = 0;
	uint8_t x = 0, row = 0;

	if (y1 % 8 == 0)
		row = y1 / 8;
	else
		row = y1 / 8 + 1;
	for (row = y0; row < y1; row++)
	{
		OLED_Set_Pos(x0, row);
		for (x = x0; x < x1; x++)
		{
			if (colorTurn)
				OLED_WR_DATA(~BMP[j++]); // 显示反相图片
			else
				OLED_WR_DATA(BMP[j++]); // 显示图片
		}
	}
}

// /**
//  * @function: void OLED_HorizontalShift(uint8_t direction)
//  * @description: 屏幕内容水平全屏滚动播放
//  * @param {uint8_t} direction			LEFT	   0x27     	RIGHT  0x26
//  * @return {*}
//  */
// void OLED_HorizontalShift(uint8_t direction)

// {
// 	OLED_WR_CMD(0x2e);		// 停止滚动
// 	OLED_WR_CMD(direction); // 设置滚动方向
// 	OLED_WR_CMD(0x00);		// 虚拟字节设置，默认为0x00
// 	OLED_WR_CMD(0x00);		// 设置开始页地址
// 	OLED_WR_CMD(0x07);		// 设置每个滚动步骤之间的时间间隔的帧频
// 	//  0x00-5帧， 0x01-64帧， 0x02-128帧， 0x03-256帧， 0x04-3帧， 0x05-4帧， 0x06-25帧， 0x07-2帧，
// 	OLED_WR_CMD(0x07); // 设置结束页地址
// 	OLED_WR_CMD(0x00); // 虚拟字节设置，默认为0x00
// 	OLED_WR_CMD(0xff); // 虚拟字节设置，默认为0xff
// 	OLED_WR_CMD(0x2f); // 开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据
// }

// /**
//  * @function: void OLED_Some_HorizontalShift(uint8_t direction,uint8_t start,uint8_t end)
//  * @description: 屏幕部分内容水平滚动播放
//  * @param {uint8_t} direction			LEFT	   0x27     	RIGHT  0x26
//  * @param {uint8_t} start 开始页地址  0x00-0x07
//  * @param {uint8_t} end  结束页地址  0x01-0x07
//  * @return {*}
//  */
// void OLED_Some_HorizontalShift(uint8_t direction, uint8_t start, uint8_t end)
// {
// 	OLED_WR_CMD(0x2e);		// 停止滚动
// 	OLED_WR_CMD(direction); // 设置滚动方向
// 	OLED_WR_CMD(0x00);		// 虚拟字节设置，默认为0x00
// 	OLED_WR_CMD(start);		// 设置开始页地址
// 	OLED_WR_CMD(0x07);		// 设置每个滚动步骤之间的时间间隔的帧频,0x07即滚动速度2帧
// 	OLED_WR_CMD(end);		// 设置结束页地址
// 	OLED_WR_CMD(0x00);		// 虚拟字节设置，默认为0x00
// 	OLED_WR_CMD(0xff);		// 虚拟字节设置，默认为0xff
// 	OLED_WR_CMD(0x2f);		// 开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据
// }

// /**
//  * @function: void OLED_VerticalAndHorizontalShift(uint8_t direction)
//  * @description: 屏幕内容垂直水平全屏滚动播放
//  * @param {uint8_t} direction				右上滚动	 0x29
//  *                                                            左上滚动   0x2A
//  * @return {*}
//  */
// void OLED_VerticalAndHorizontalShift(uint8_t direction)
// {
// 	OLED_WR_CMD(0x2e);		// 停止滚动
// 	OLED_WR_CMD(direction); // 设置滚动方向
// 	OLED_WR_CMD(0x01);		// 虚拟字节设置
// 	OLED_WR_CMD(0x00);		// 设置开始页地址
// 	OLED_WR_CMD(0x07);		// 设置每个滚动步骤之间的时间间隔的帧频，即滚动速度
// 	OLED_WR_CMD(0x07);		// 设置结束页地址
// 	OLED_WR_CMD(0x01);		// 垂直滚动偏移量
// 	OLED_WR_CMD(0x00);		// 虚拟字节设置，默认为0x00
// 	OLED_WR_CMD(0xff);		// 虚拟字节设置，默认为0xff
// 	OLED_WR_CMD(0x2f);		// 开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据
// }

/**
 * @function: void OLED_DisplayMode(uint8_t mode)
 * @description: 屏幕内容取反显示
 * @param {uint8_t} direction ON 0xA7; OFF 0xA6	默认此模式，设置像素点亮
 * @return {*}
 */
void OLED_DisplayMode(uint8_t mode)
{
	OLED_WR_CMD(mode);
}

/**
 * @function: void OLED_IntensityControl(uint8_t intensity)
 * @description: 屏幕亮度调节
 * @param  {uint8_t} intensity	0x00-0xFF,RESET=0x7F
 * @return {*}
 */
void OLED_IntensityControl(uint8_t intensity)
{
	OLED_WR_CMD(0x81);
	OLED_WR_CMD(intensity);
}
