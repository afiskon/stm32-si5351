# STM32 & Si5351 Signal Generator

A signal generator based on STM32F103 MCU, Si5351 and 2004 I2C LCD.

MCU pinout:

* PB8, PB9 = I2C bus SCL, SDA. Connect to LCD and Si5351
* PA8, PA9 = rotary encoder
* PB10 = rotary encoder button
* PA3, PA4 = two more buttons

Article (in Russian): https://eax.me/si5351-signal-generator/
