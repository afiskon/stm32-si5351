// vim: set ai et ts=4 sw=4:

// change for your chip
#include "stm32f1xx_hal.h"
#include <lcd.h>

// change if necessary
#define I2C_HANDLE hi2c1
extern I2C_HandleTypeDef I2C_HANDLE;

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

#define LCD_DELAY_MS 5

uint8_t lcd_i2c_addr = (0x27 << 1);

HAL_StatusTypeDef LCD_SendInternal(uint8_t data, uint8_t flags) {
    HAL_StatusTypeDef res;
    for(;;) {
        res = HAL_I2C_IsDeviceReady(&I2C_HANDLE, lcd_i2c_addr, 1, HAL_MAX_DELAY);
        if(res == HAL_OK)
            break;
    }

    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    uint8_t data_arr[4];
    data_arr[0] = up|flags|BACKLIGHT|PIN_EN;
    data_arr[1] = up|flags|BACKLIGHT;
    data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
    data_arr[3] = lo|flags|BACKLIGHT;

    res = HAL_I2C_Master_Transmit(&I2C_HANDLE, lcd_i2c_addr, data_arr,
                                  sizeof(data_arr), HAL_MAX_DELAY);
    HAL_Delay(LCD_DELAY_MS);
    return res;
}

void LCD_SendCommand(uint8_t cmd) {
    LCD_SendInternal(cmd, 0);
}

void LCD_SendData(uint8_t data) {
    LCD_SendInternal(data, PIN_RS);
}

void LCD_Clear() {
    LCD_SendCommand(0b00000001);
}

void LCD_Goto(int8_t line, int8_t chnum) {
    if(chnum < 0) {
        chnum = 0;
    } else if(chnum > 19) {
        chnum = 19;
    }

    if(line == 0) {
        LCD_SendCommand(0b10000000 + chnum);
    } else if(line == 1) {
        LCD_SendCommand(0b11000000 + chnum);
    } else if(line == 2) {
        LCD_SendCommand(0b10000000 + 20 + chnum);
    } else if(line == 3) {
        LCD_SendCommand(0b11000000 + 20 + chnum);
    }
}

void LCD_UnderlineEnabled(bool enabled) {
    if(enabled) {
        // display on, underline on, blink off
        LCD_SendCommand(0b00001110);
    } else {
        // display on, underline off, blink off
        LCD_SendCommand(0b00001100);
    }   
}

void LCD_Init() {
    // Check for an alternative LCD address
    HAL_StatusTypeDef res;
    res = HAL_I2C_IsDeviceReady(&I2C_HANDLE, (0x3F << 1), 1, HAL_MAX_DELAY);
    if(res == HAL_OK) {
        lcd_i2c_addr = (0x3F << 1);
    }

    // 4-bit mode, 2 lines, 5x7 format
    LCD_SendCommand(0b00110000);
    // display & cursor home (keep this!)
    LCD_SendCommand(0b00000010);
    LCD_UnderlineEnabled(true);
    LCD_Clear();

    // create 8 custom characters for the S-meter
    for(uint8_t ch = 0; ch < 8; ch++) {
        // set CGRAM Address
        LCD_SendCommand(0b01000000 + (ch << 3));
        for(uint8_t line = 0; line < 8; line++) {
            if(ch >= (7-line)) {
                LCD_SendData(0b11111);
            } else {
                LCD_SendData(0b00000);
            }
        }
    }
}

void LCD_SendString(char *str) {
    while(*str) {
        LCD_SendData((uint8_t)(*str));
        str++;
    }
}
