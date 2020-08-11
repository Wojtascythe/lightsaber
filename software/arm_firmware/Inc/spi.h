/**
  ******************************************************************************
  * File Name          : SPI.h
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __spi_H
#define __spi_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
// /*##################### SPI2 ###################################*/
// #define DISCOVERY_SPIx                          SPI2
// #define DISCOVERY_SPIx_CLOCK_ENABLE()           __HAL_RCC_SPI2_CLK_ENABLE()
// #define DISCOVERY_SPIx_CLOCK_DISABLE()          __HAL_RCC_SPI2_CLK_DISABLE()
// #define DISCOVERY_SPIx_GPIO_PORT                GPIOD                      /* GPIOD */
// #define DISCOVERY_SPIx_AF                       GPIO_AF5_SPI2
// #define DISCOVERY_SPIx_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOD_CLK_ENABLE()
// #define DISCOVERY_SPIx_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOD_CLK_DISABLE()
// #define DISCOVERY_SPIx_GPIO_FORCE_RESET()       __HAL_RCC_SPI2_FORCE_RESET()
// #define DISCOVERY_SPIx_GPIO_RELEASE_RESET()     __HAL_RCC_SPI2_RELEASE_RESET()
// #define DISCOVERY_SPIx_SCK_PIN                  GPIO_PIN_1                 /* PD.01*/
// #define DISCOVERY_SPIx_MISO_PIN                 GPIO_PIN_3                 /* PD.03 */
// #define DISCOVERY_SPIx_MOSI_PIN                 GPIO_PIN_4                 /* PD.04 */
//
// /* Maximum Timeout values for flags waiting loops. These timeouts are not based
//    on accurate values, they just guarantee that the application will not remain
//    stuck if the SPI communication is corrupted.
//    You may modify these timeout values depending on CPU frequency and application
//    conditions (interrupts routines ...). */
// #define SPIx_TIMEOUT_MAX                        ((uint32_t)0x1000)
// /* Read/Write command */
// #define READWRITE_CMD                           ((uint8_t)0x80)
// /* Multiple byte read/write command */
// #define MULTIPLEBYTE_CMD                        ((uint8_t)0x40)
// /* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
// #define DUMMY_BYTE
// /* LL definition */
// #define __SPI_DIRECTION_2LINES(__HANDLE__)   do{\
//                                              CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
//                                              }while(0);
//
// #define __SPI_DIRECTION_2LINES_RXONLY(__HANDLE__)   do{\
//                                                    CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
//                                                    SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY);\
//                                                    }while(0);
//
// #define __SPI_DIRECTION_1LINE_TX(__HANDLE__) do{\
//                                              CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
//                                              SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
//                                              }while(0);
//
// #define __SPI_DIRECTION_1LINE_RX(__HANDLE__) do {\
//                                              CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
//                                              SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_BIDIMODE);\
//                                              } while(0);((uint8_t)0x00)
/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
