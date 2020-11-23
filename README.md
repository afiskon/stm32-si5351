# stm32-si5351

HAL-based Si5351 driver for STM32.

Si5351 is a I2C-programmable 8 kHz - 160 MHz clock generator made by Silicon Labs. It has 3 ports (or more depending on modification) with 50 Ohm output impedance. The signal level can be changed in ~2-11 dBm range.

Basic interface:

```
const int32_t correction = 978;
si5351_Init(correction);

// 28 MHz @ ~7 dBm
si5351_SetupCLK0(28000000, SI5351_DRIVE_STRENGTH_4MA);

// 144 MHz @ ~7 dBm
si5351_SetupCLK2(144000000, SI5351_DRIVE_STRENGTH_4MA);

// Enable CLK0 and CLK2
si5351_EnableOutputs((1<<0) | (1<<2));
```

Advanced interface:

```
const int32_t correction = 978;
si5351_Init(correction);

si5351PLLConfig_t pll_conf;
si5351OutputConfig_t out_conf;
int32_t Fclk = 7000000; // 7 MHz

si5351_Calc(Fclk, &pll_conf, &out_conf);
si5351_SetupPLL(SI5351_PLL_A, &pll_conf);
si5351_SetupOutput(0, SI5351_PLL_A, SI5351_DRIVE_STRENGTH_4MA, &out_conf);
si5351_EnableOutputs(1<<0);
```

More comments are in the code. See also examples/ directory.

This library was forked from [ProjectsByJRP/si5351-stm32](https://github.com/ProjectsByJRP/si5351-stm32) which in it's turn is a port of [adafruit/Adafruit_Si5351_Library](https://github.com/adafruit/Adafruit_Si5351_Library).
