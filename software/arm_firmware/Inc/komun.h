/*
 * komun.h
 *
 *  Created on: 16.04.2018
 *      Author: wojta
 */

#ifndef KOMUN_H_
#define KOMUN_H_

#include "stm32l4xx_hal.h"
// DEFINICJE DO SPI!!!!!!!!!!!!!!!!
#define SPIaz2                          SPI2
#define SPIaz2_CLOCK_ENABLE()           __HAL_RCC_SPI2_CLK_ENABLE()
#define SPIaz2_CLOCK_DISABLE()          __HAL_RCC_SPI2_CLK_DISABLE()
#define SPIaz2_GPIO_PORT                GPIOD
#define SPIaz2_AF                       GPIO_AF5_SPI2
#define SPIaz2_SCK_PIN                  GPIO_PIN_1
#define SPIaz2_MISO_PIN                 GPIO_PIN_3
#define SPIaz2_MOSI_PIN                 GPIO_PIN_4
#define SPIaz2_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOD_CLK_ENABLE()
#define SPIaz2_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOD_CLK_DISABLE()
#define SPIaz2_GPIO_FORCE_RESET()       __HAL_RCC_SPI2_FORCE_RESET()
#define SPIaz2_GPIO_RELEASE_RESET()     __HAL_RCC_SPI2_RELEASE_RESET()



//Ciekawa sprawa. Definicja maksymalnego czasu spêdzonego w przerwaniu. Ma to zapobiegac utkniêciu aplikacji w przerwaniu, gdy dojdzie do b³êdu komunikacji sPI
#define SPIx_TIMEOUT_MAX                        ((uint32_t)0x1000)
//Odczytuj, zapisuj
#define READWRITE_CMD                           ((uint8_t)0x80)
//Odczytuj, zapisuj wiele bitów
#define MULTIPLEBYTE_CMD                        ((uint8_t)0x40)
//Dummy bite. Chodzi o to by urz¹dzenie Master w komunikacji sPI wys³a³o jakiœ bit do Slave'a jako rozkaz odpowiedniej konfiguracji zegara dla slave'a
#define DUMMY_BYTE                              ((uint8_t)0x00)



//AKCELEROMETR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//Piny do wyboru dzia³ania chipu akcelerometru
#define ACCELERO_CS_LOW()                       HAL_GPIO_WritePin(ACCELERO_CS_GPIO_PORT, ACCELERO_CS_PIN, GPIO_PIN_RESET)
#define ACCELERO_CS_HIGH()                      HAL_GPIO_WritePin(ACCELERO_CS_GPIO_PORT, ACCELERO_CS_PIN, GPIO_PIN_SET)

//Piny do interfejsu komunikacji SPI akcelerometru
#define ACCELERO_CS_GPIO_PORT                   GPIOE
#define ACCELERO_CS_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define ACCELERO_CS_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOE_CLK_DISABLE()
#define ACCELERO_CS_PIN                         GPIO_PIN_0

//Piny do przerwañ akcelerometru
#define ACCELERO_XLINT_GPIO_PORT                  GPIOE
#define ACCELERO_XLINT_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOE_CLK_ENABLE()
#define ACCELERO_XLINT_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOE_CLK_DISABLE()
#define ACCELERO_XLINT_PIN                       GPIO_PIN_1
#define ACCELERO_XLINT_EXTI_IRQn                 EXTI1_IRQn

//¯YROSKOP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//Piny do wyboru dzia³ania chipu ¿yroskopu
#define GYRO_CS_LOW()                           HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()                          HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_SET)


//Piny do interfejsu komunikacji SPI ¿yroskopu
#define GYRO_CS_GPIO_PORT                       GPIOD
#define GYRO_CS_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()
#define GYRO_CS_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOD_CLK_DISABLE()
#define GYRO_CS_PIN                             GPIO_PIN_7


//Piny do przerwañ ¿yroskopu
#define GYRO_INT1_GPIO_PORT                     GPIOD
#define GYRO_INT1_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOD_CLK_ENABLE()
#define GYRO_INT1_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOD_CLK_DISABLE()
#define GYRO_INT1_PIN                           GPIO_PIN_2
#define GYRO_INT1_EXTI_IRQn                     EXTI2_IRQn
#define GYRO_INT2_GPIO_PORT                     GPIOB
#define GYRO_INT2_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOB_CLK_ENABLE()
#define GYRO_INT2_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOB_CLK_DISABLE()
#define GYRO_INT2_PIN                           GPIO_PIN_8
#define GYRO_INT2_EXTI_IRQn                     EXTI9_5_IRQn







#endif /* KOMUN_H_ */
