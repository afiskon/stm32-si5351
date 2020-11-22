// vim: set ai et ts=4 sw=4:
#ifndef _SI5351_H_
#define _SI5351_H_

typedef enum {
    SI5351_PLL_A = 0,
    SI5351_PLL_B,
} si5351PLL_t;

typedef enum {
    SI5351_R_DIV_1   = 0,
    SI5351_R_DIV_2   = 1,
    SI5351_R_DIV_4   = 2,
    SI5351_R_DIV_8   = 3,
    SI5351_R_DIV_16  = 4,
    SI5351_R_DIV_32  = 5,
    SI5351_R_DIV_64  = 6,
    SI5351_R_DIV_128 = 7,
} si5351RDiv_t;

typedef enum {
    SI5351_DRIVE_STRENGTH_2MA = 0x00, //  ~ 2.2 dBm
    SI5351_DRIVE_STRENGTH_4MA = 0x01, //  ~ 7.5 dBm
    SI5351_DRIVE_STRENGTH_6MA = 0x02, //  ~ 9.5 dBm
    SI5351_DRIVE_STRENGTH_8MA = 0x03, // ~ 10.7 dBm
} si5351DriveStrength_t;

typedef struct {
    int32_t mult;
    int32_t num;
    int32_t denom;
} si5351PLLConfig_t;

typedef struct {
    int32_t div;
    int32_t num;
    int32_t denom;
    si5351RDiv_t rdiv;
} si5351OutputConfig_t;

// Basic interface allows to use only CLK0 and CLK2.
// This interface uses separate PLLs for both CLK0 and CLK2 thus the frequencies
// can be changed independelty. If you also need CLK1 one PLL should
// be shared between two CLKx and things get a little more complicated.
// CLK0 and CLK2 were chosen because they are distant from each other on a common
// Si5351 module which makes using them a little more convenient than CLK0 and CLK1.
void si5351_Init(int32_t correction);
void si5351_SetupCLK0(int32_t Fclk, si5351DriveStrength_t driveStrength);
void si5351_SetupCLK2(int32_t Fclk, si5351DriveStrength_t driveStrength);
void si5351_EnableOutputs(uint8_t enabled);

// Advanced interface. Use it if you need CLK0, CLK1 and CLK2 simultaneously.
// You can use the knowlage that si5351_Calc always uses 900 Meg PLL for frequencies below 81 Megs.
// This PLL can safely be shared between all CLKx that work @ <= 81 Megs.
// You can also modify si5351.c to share one PLL for any frequencies <= 112.5 Megs,
// however this will increase the worse case calculation error to 13 Hz.
void si5351_Calc(int32_t Fclk, si5351PLLConfig_t* pll_conf, si5351OutputConfig_t* out_conf);
void si5351_SetupPLL(si5351PLL_t pll, si5351PLLConfig_t* conf);
void si5351_SetupOutput(uint8_t output, si5351PLL_t pllSource, si5351DriveStrength_t driveStength, si5351OutputConfig_t* conf);

#endif
