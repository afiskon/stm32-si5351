// vim: set ai et ts=4 sw=4:
#ifndef _LCD_H_
#define _LCD_H_

#include <stdbool.h>

void LCD_Init();
void LCD_Clear();
void LCD_Goto(int8_t line, int8_t chnum);
void LCD_UnderlineEnabled(bool enabled);
void LCD_SendString(char *str);

#endif