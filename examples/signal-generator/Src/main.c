/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "si5351.h"
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define CLK0 0
#define CLK2 2
const int32_t si5351_correction = 5245;

typedef enum {
	CURSOR_CHANGING_FREQUENCY = 0,
	CURSOR_CHANGING_POWER,
} CursorChanging_t;

typedef struct {
	uint8_t x;
	uint8_t y;
	uint8_t channel;
	uint32_t frequencyStep;
	CursorChanging_t changing;
} CursorState_t;

const CursorState_t CURSOR_STATES[] = {
	{
		.x = 9, .y = 0, .frequencyStep = 100000000,
		.channel = 0, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 10, .y = 0, .frequencyStep = 10000000,
		.channel = 0, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 11, .y = 0, .frequencyStep = 1000000,
		.channel = 0, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 13, .y = 0, .frequencyStep = 100000,
		.channel = 0, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 14, .y = 0, .frequencyStep = 10000,
		.channel = 0, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 15, .y = 0, .frequencyStep = 1000,
		.channel = 0, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 17, .y = 0, .frequencyStep = 100,
		.channel = 0, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 18, .y = 0, .frequencyStep = 10,
		.channel = 0, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 19, .y = 0, .frequencyStep = 1,
		.channel = 0, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 11, .y = 1, .frequencyStep = 0,
		.channel = 0, .changing = CURSOR_CHANGING_POWER,
	},
	{
		.x = 9, .y = 2, .frequencyStep = 100000000,
		.channel = 1, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 10, .y = 2, .frequencyStep = 10000000,
		.channel = 1, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 11, .y = 2, .frequencyStep = 1000000,
		.channel = 1, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 13, .y = 2, .frequencyStep = 100000,
		.channel = 1, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 14, .y = 2, .frequencyStep = 10000,
		.channel = 1, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 15, .y = 2, .frequencyStep = 1000,
		.channel = 1, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 17, .y = 2, .frequencyStep = 100,
		.channel = 1, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 18, .y = 2, .frequencyStep = 10,
		.channel = 1, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 19, .y = 2, .frequencyStep = 1,
		.channel = 1, .changing = CURSOR_CHANGING_FREQUENCY,
	},
	{
		.x = 11, .y = 3, .frequencyStep = 0,
		.channel = 1, .changing = CURSOR_CHANGING_POWER,
	},
};

typedef struct {
	bool enabled;
	si5351DriveStrength_t driveStrength;
} PowerState_t;

const PowerState_t POWER_STATES[] = {
	{ .enabled = false, .driveStrength = SI5351_DRIVE_STRENGTH_2MA },
	{ .enabled = true,  .driveStrength = SI5351_DRIVE_STRENGTH_2MA },
	{ .enabled = true,  .driveStrength = SI5351_DRIVE_STRENGTH_4MA },
	{ .enabled = true,  .driveStrength = SI5351_DRIVE_STRENGTH_6MA },
	{ .enabled = true,  .driveStrength = SI5351_DRIVE_STRENGTH_8MA },
};

typedef struct {
	int32_t frequency;
	int8_t powerState;
} ChannelState_t;

int32_t currentCursorState = 0;
uint8_t modeIQenabled = 0;
ChannelState_t currentChannelStates[2] = {
	{
		.frequency = 10000000,
		.powerState = 0,
	},
	{
		.frequency = 10000000,
		.powerState = 0,
	},
};

void displayCurrentChannelStates() {
	char buff[16];
	for(int i = 0; i < 2; i++) {
		LCD_Goto(i*2, 9);
		if((i == 1) && modeIQenabled) {
			LCD_SendString("=CH1+90\xDF   ");
		} else {
			snprintf(buff, sizeof(buff), "%03ld.%03ld.%03ld",
				currentChannelStates[i].frequency / 1000000,
				(currentChannelStates[i].frequency / 1000) % 1000,
				currentChannelStates[i].frequency % 1000);
			LCD_SendString(buff);
		}
		
		LCD_Goto(i*2 + 1, 9);
		if(!POWER_STATES[currentChannelStates[i].powerState].enabled) {
			LCD_SendString("OFF");
		} else if(POWER_STATES[currentChannelStates[i].powerState].driveStrength == SI5351_DRIVE_STRENGTH_2MA) {
			LCD_SendString("2MA");
		} else if(POWER_STATES[currentChannelStates[i].powerState].driveStrength == SI5351_DRIVE_STRENGTH_4MA) {
			LCD_SendString("4MA");
		} else if(POWER_STATES[currentChannelStates[i].powerState].driveStrength == SI5351_DRIVE_STRENGTH_6MA) {
			LCD_SendString("6MA");
		} else {
			LCD_SendString("8MA");
		}
	}
}

void updateSi5351Parameters() {
	si5351PLLConfig_t pll_conf;
	si5351OutputConfig_t out_conf;
	uint8_t enabledOutputs = 0;

    si5351_EnableOutputs(0);

    if(modeIQenabled) {
		si5351_CalcIQ(currentChannelStates[0].frequency, &pll_conf, &out_conf);

		uint8_t phaseOffset = (uint8_t)out_conf.div;
		si5351_SetupOutput(0, SI5351_PLL_A, POWER_STATES[currentChannelStates[0].powerState].driveStrength, &out_conf, 0);
		si5351_SetupOutput(2, SI5351_PLL_A, POWER_STATES[currentChannelStates[1].powerState].driveStrength, &out_conf, phaseOffset);
		si5351_SetupPLL(SI5351_PLL_A, &pll_conf);
    } else {
		si5351_Calc(currentChannelStates[0].frequency, &pll_conf, &out_conf);
		si5351_SetupOutput(0, SI5351_PLL_A, POWER_STATES[currentChannelStates[0].powerState].driveStrength, &out_conf, 0);
		si5351_SetupPLL(SI5351_PLL_A, &pll_conf);

		si5351_Calc(currentChannelStates[1].frequency, &pll_conf, &out_conf);
		si5351_SetupOutput(2, SI5351_PLL_B, POWER_STATES[currentChannelStates[1].powerState].driveStrength, &out_conf, 0);
		si5351_SetupPLL(SI5351_PLL_B, &pll_conf);
	}

	if(POWER_STATES[currentChannelStates[0].powerState].enabled) {
		enabledOutputs |= (1 << CLK0);
	}

	if(POWER_STATES[currentChannelStates[1].powerState].enabled) {
		enabledOutputs |= (1 << CLK2);
	}

	si5351_EnableOutputs(enabledOutputs);
}

#define BUTTON_DEBOUNCE_TIME_MS 300
typedef enum {
    BUTTON_PRESSED = 0,
    BUTTON_RELEASED = 1,
    BUTTON_DEBOUNCE = 2,
} ButtonStatus_t;

ButtonStatus_t buttonRotEncoderPressed(uint32_t now) {
    static uint32_t lastPressed = 0;
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) {
        if(now - lastPressed > BUTTON_DEBOUNCE_TIME_MS) {
            lastPressed = now;
            return BUTTON_PRESSED;
        } else {
            return BUTTON_DEBOUNCE;
        }
    }
    return BUTTON_RELEASED;
}

bool buttonLeftPressedSimple() {
    return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET);
}

ButtonStatus_t buttonLeftPressed(uint32_t now) {
    static uint32_t lastPressed = 0;
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
        if(now - lastPressed > BUTTON_DEBOUNCE_TIME_MS) {
            lastPressed = now;
            return BUTTON_PRESSED;
        } else {
            return BUTTON_DEBOUNCE;
        }
    }
    return BUTTON_RELEASED;
}

bool buttonRightPressedSimple() {
    return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET);
}

ButtonStatus_t buttonRightPressed(uint32_t now) {
    static uint32_t lastPressed = 0;
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {
        if(now - lastPressed > BUTTON_DEBOUNCE_TIME_MS) {
            lastPressed = now;
            return BUTTON_PRESSED;
        } else {
            return BUTTON_DEBOUNCE;
        }
    }
    return BUTTON_RELEASED;
}

#define LOOP_DELAY 5

// returns 0 if rotary encoder was not rotated
int32_t getRotaryEncoderDelta() {
    static int32_t prevCounter = 0;
    int32_t currCounter = __HAL_TIM_GET_COUNTER(&htim1);
    currCounter = 32767 - ((currCounter-1) & 0xFFFF) / 2;
    if(currCounter > 32768/2) {
        // convert ... 32766, 32767, 0, 1, 2 ... into:
        //               ... -2, -1, 0, 1, 2 ...
        // this simplifies `delta` calculation
        currCounter = currCounter-32768;
    }
    if(currCounter != prevCounter) {
        int32_t delta = currCounter-prevCounter;
        prevCounter = currCounter;
        // debounce or skip counter overflow
        if((delta > -10) && (delta < 10)) {
            return delta;
        }
    }

    return 0;
}

void init() {
    LCD_Init();
    LCD_UnderlineEnabled(false);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

    if(buttonLeftPressedSimple() && buttonRightPressedSimple()) {
		// perform I2C scan
		LCD_Goto(0, 0);
		LCD_SendString("I2C Scan:");
		LCD_Goto(1, 0);
		for(uint16_t i = 0; i < 128; i++) {
			HAL_StatusTypeDef res;
			res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10);
			if(res == HAL_OK) {
				char msg[64];
				snprintf(msg, sizeof(msg), "%02X ", i);
				LCD_SendString(msg);
			}
		}  
		HAL_Delay(3000);
		LCD_Clear();
	}

    LCD_Goto(0, 0);
    LCD_SendString("   STM32 & Si5351   ");
    LCD_Goto(1, 0);
    LCD_SendString("  Signal Generator  ");
    LCD_Goto(2, 0);
    LCD_SendString("   by Alex, R2AUK   ");
    LCD_Goto(3, 0);
    LCD_SendString("      Sep 2021      ");
    HAL_Delay(3000);
    LCD_Clear();

    LCD_Goto(0, 0);
    LCD_SendString("CH1 Freq ???.???.???");
    LCD_Goto(1, 0);
    LCD_SendString("    Pwr  ???        ");
    LCD_Goto(2, 0);
    LCD_SendString("CH2 Freq ???.???.???");
    LCD_Goto(3, 0);
    LCD_SendString("    Pwr  ???        ");
    LCD_Goto(0, 11);
    LCD_UnderlineEnabled(true);

    si5351_Init(si5351_correction);
	updateSi5351Parameters();
	displayCurrentChannelStates();
}

void loop() {
	static uint32_t lastModeChangeTime = 0;
	uint32_t now = HAL_GetTick();
	int32_t delta = getRotaryEncoderDelta();

	if(delta != 0) {
		uint8_t ch = CURSOR_STATES[currentCursorState].channel;
		switch(CURSOR_STATES[currentCursorState].changing) {
		case CURSOR_CHANGING_FREQUENCY:
			currentChannelStates[ch].frequency += delta*CURSOR_STATES[currentCursorState].frequencyStep;
			if(modeIQenabled) {
				if(currentChannelStates[ch].frequency < 1400000) {
					currentChannelStates[ch].frequency = 1400000;
				} else if(currentChannelStates[ch].frequency > 100000000) {
					currentChannelStates[ch].frequency = 100000000;
				}
			} else {
				if(currentChannelStates[ch].frequency < 8000) {
					currentChannelStates[ch].frequency = 8000;
				} else if(currentChannelStates[ch].frequency > 160000000) {
					currentChannelStates[ch].frequency = 160000000;
				}
			}
			break;
		case CURSOR_CHANGING_POWER:
			currentChannelStates[ch].powerState += delta;
			if(currentChannelStates[ch].powerState < 0) {
				currentChannelStates[ch].powerState = sizeof(POWER_STATES)/sizeof(POWER_STATES[0]) - 1;
			} else if(currentChannelStates[ch].powerState >= sizeof(POWER_STATES)/sizeof(POWER_STATES[0])) {
				currentChannelStates[ch].powerState = 0;
			}

			break;
		}

		updateSi5351Parameters();
		displayCurrentChannelStates();
	}

	if(!modeIQenabled && (buttonRotEncoderPressed(now) == BUTTON_PRESSED)) {
		int32_t threshold = (sizeof(CURSOR_STATES)/sizeof(CURSOR_STATES[0])) / 2;
		if(currentCursorState >= threshold) {
			currentCursorState -= threshold;
		} else {
			currentCursorState += threshold;
		}
	}

	if(buttonLeftPressedSimple() && buttonRightPressedSimple() && (now - lastModeChangeTime > BUTTON_DEBOUNCE_TIME_MS)) {
		modeIQenabled = !modeIQenabled;
		currentChannelStates[0].powerState = 0; // turn OFF both channels
		currentChannelStates[1].powerState = 0;
		lastModeChangeTime = now;

		if(modeIQenabled) {
			currentCursorState = 0;

			if(currentChannelStates[0].frequency < 1400000) {
				currentChannelStates[0].frequency = 1400000;
			} else if(currentChannelStates[0].frequency > 100000000) {
				currentChannelStates[0].frequency = 100000000;
			}
		}

		updateSi5351Parameters();
		displayCurrentChannelStates();
	} else {
		uint8_t leftPressed = (buttonLeftPressed(now) == BUTTON_PRESSED);
		uint8_t rightPressed = (buttonRightPressed(now) == BUTTON_PRESSED);

		do {
			if(leftPressed) {
				currentCursorState--;
			} else if(rightPressed) {
				currentCursorState++;
			}

			if(currentCursorState < 0) {
				currentCursorState = sizeof(CURSOR_STATES)/sizeof(CURSOR_STATES[0]) - 1;
			}

			if(currentCursorState >= sizeof(CURSOR_STATES)/sizeof(CURSOR_STATES[0])) {
				currentCursorState = 0;
			}
		} while(modeIQenabled &&
			(CURSOR_STATES[currentCursorState].channel == 1) && 
			(CURSOR_STATES[currentCursorState].changing == CURSOR_CHANGING_FREQUENCY));
	}

	LCD_Goto(CURSOR_STATES[currentCursorState].y, CURSOR_STATES[currentCursorState].x);

    HAL_Delay(LOOP_DELAY);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    loop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
