
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "dfsdm.h"
#include "dma.h"
#include "i2c.h"
#include "sai.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "gyros.h"
#include "accel.h"

#include "stm32l476g_discovery_gyroscope.h"
#include "stm32l476g_discovery_compass.h"
#include "stm32l476g_discovery_audio.h"

#include "math.h"
#include "sample.h"

#define BASE_FREQ 400.0f
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t Received[10];
int32_t RemainingAudioSamplesNb;
volatile uint16_t * frameData, *genData;
uint16_t buffor1[800], buffor2[800];
volatile float cc = 0.0;
volatile uint8_t audioFlag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void AudioPlay_TransferComplete_CallBack() {
	if (audioFlag == 1) {
		if (frameData == buffor1) {
			frameData = buffor2;
		} else {
			frameData = buffor1;
		}
		audioFlag = 0;
	}

//	if(cc > 2.0){
//				frameData = light_saber_sample_move;
//
//			}else{
//				frameData = light_saber_sample_idle;
//
//			}


	if (BSP_AUDIO_OUT_Play(frameData, RemainingAudioSamplesNb) != AUDIO_OK) {
		HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, SET);
		Error_Handler();
	} else {
		HAL_GPIO_TogglePin(LD_G_GPIO_Port, LD_G_Pin);
	}

}
void AudioPlay_Error_CallBack(void) {
	/* Stop the program with an infinite loop */
	HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, SET);
	Error_Handler();
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float sampleRate = 8000.0; // hertz
	float freq = BASE_FREQ;         // hertz
	float duration = 0.1;       // seconds

	/* Gyroscope variables */
	uint8_t message[100], characters_count;
	int16_t buffer[3];
	float x, y, z;
	float nSamples = (duration * sampleRate);
	uint16_t i;
	uint8_t wiadom[100];
	uint8_t char_count;
	int16_t buffa[3] = { 0 };
	float buffg[3] = { 0 };
	float xa, ya, za, va = 0.0, xaa = 0.0, yaa = 0.0, zaa = 0.0;
	float xg, yg, zg;
	uint32_t interval = 0, prev = 0, current = 0;

	frameData = buffor1;
	//frameData = light_saber_sample;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_SAI1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_DFSDM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, RESET);
	HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, RESET);

	for (i = 0; i < (uint32_t) nSamples; ++i) {
		buffor1[i] = (uint16_t) ((sin(
				freq * (float) i * 6.28 / sampleRate) + 1.0) * 32000);
	}


	if (ZYROS_Init() != Zyros_ok) {
		HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, SET);
		/* Initialization Error */
		Error_Handler();
	}

	if (AKCEL_Init() != Akcel_ok) {
		HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, SET);
		/* Initialization Error */
		Error_Handler();
	}

	RemainingAudioSamplesNb = 800; //sizeof(light_saber_sample)/sizeof(uint16_t); //1600;//(uint32_t)nSamples;
	HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, RESET);
	if (BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 60, 8000) != AUDIO_OK) {
		HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, SET);
		Error_Handler();       //dajemy sygnal error
	}

	BSP_AUDIO_OUT_RegisterCallbacks(AudioPlay_Error_CallBack,
	NULL, AudioPlay_TransferComplete_CallBack);

	if (BSP_AUDIO_OUT_Play(frameData, RemainingAudioSamplesNb) != AUDIO_OK) {
		HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, SET);

		Error_Handler();
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		AKCEL_GetXYZ(buffa);
		xa = ((float) buffa[0] /4096.0);
		ya = ((float) buffa[1] /4096.0);
		za = ((float) buffa[2] /4096.0);

		char_count = sprintf((char *) wiadom, "Xa: %f Ya: %f Za: %f\n", xa, ya,
				za);
		HAL_UART_Transmit(&huart2, wiadom, char_count, 0xFFFF);

		ZYROS_GetXYZ(buffg);
		xg = ((float) buffg[0] * 0.244 / 1000.0);
		yg = ((float) buffg[1] * 0.244 / 1000.0);
		zg = ((float) buffg[2] * 0.244 / 1000.0);

		char_count = sprintf((char *) wiadom, "Xg: %f Yg: %f Zg: %f\n", xg, yg,
				zg);
		HAL_UART_Transmit(&huart2, wiadom, char_count, 0xFFFF);


		current = HAL_GetTick();
					interval = current - prev;
					prev = current;
		xaa += xa * ((float)interval / 1000.0);
		yaa += ya * ((float)interval/ 1000.0);
		zaa += za * ((float)interval/ 1000.0);
		va = sqrt(xaa*xaa + yaa*yaa + zaa*zaa);
		char_count = sprintf((char *) wiadom, "V: %f\n", va);
				HAL_UART_Transmit(&huart2, wiadom, char_count, 0xFFFF);

				if(audioFlag == 0){
					freq = (340.0/(340.0-va)*BASE_FREQ);
					char_count = sprintf((char *) wiadom, "f: %f\n", freq);
									HAL_UART_Transmit(&huart2, wiadom, char_count, 0xFFFF);
					if(frameData == buffor1){
						genData = buffor2;
					}else{
						genData = buffor1;
					}
					for(i=0; i<nSamples; ++i ){
					          genData[i] = (uint16_t)((sin(freq*(float)i*6.28/sampleRate)+1.0)*32000);

					      }
					audioFlag = 1;
		}


		//cc = sqrt(xa*xa+ya*ya+za*za);


	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Convert value to display string
 * @param  None
 * @retval None
 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
