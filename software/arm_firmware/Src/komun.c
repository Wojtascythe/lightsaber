/*
 * komun.c
 *
 *  Created on: 16.04.2018
 *      Author: wojta
 */

#include "komun.h"
#define __SPI_DIRECTION_2LINES(__HANDLE__)   do{\
                                             CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
                                             }while(0);

#define __SPI_DIRECTION_2LINES_RXONLY(__HANDLE__)   do{\
                                                   CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
                                                   SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY);\
                                                   }while(0);

#define __SPI_DIRECTION_1LINE_TX(__HANDLE__) do{\
                                             CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
                                             SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
                                             }while(0);

#define __SPI_DIRECTION_1LINE_RX(__HANDLE__) do {\
                                             CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);\
                                             SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_BIDIMODE);\
                                             } while(0);



uint32_t SpixTimeout = SPIx_TIMEOUT_MAX;           //Maksymalny czas wp rzerwaniu w jakim moze byc,
//													je�li posz�o co� nie tak i SPI zawali�o, to wywala aplikacj� z przerwania
static SPI_HandleTypeDef SpiHandle;

//Funkcje do komunikacji SPI
static void               SPIx_Init(void);
static void               SPIx_MspInit(SPI_HandleTypeDef *hspi);
static void               SPIx_DeInit(void);
static void               SPIx_MspDeInit(void);
static uint8_t            SPIx_WriteRead(uint8_t Byte);
static void               SPIx_Write(uint8_t byte);
static uint8_t            SPIx_Read(void);

//Funkcje do komunikacji z akcelerometrem
void                      AKCEL_SPI_INIT(void);
void                      AKCEL_SPI_DEINIT(void);
void                      AKCEL_SPI_ITCONFIG(void);
void                      AKCEL_SPI_WRITE(uint8_t RegisterAddr, uint8_t Value);
uint8_t                   AKCEL_SPI_READ(uint8_t RegisterAddr);

//Funkcje do komunikacji z �yroskopem
void                      ZYROS_SPI_INIT(void);
void                      ZYROS_SPI_DEINIT(void);
void                      ZYROS_SPI_WRITE(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void                      ZYROS_SPI_READ(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);


 void SPIx_Init(void)
{
  if (HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_RESET)
  {
    //Konfigurujemy SPI
    SpiHandle.Instance = SPIaz2;

    //Dobrze dac zegar na 80MHz, bo lsm303c w komunikacji SPI odczytuje i zapisuje dane w maksymalnie 10MHz (80/8=10)
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    SpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial = 7;
    SpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    SpiHandle.Init.Mode = SPI_MODE_MASTER;

    SPIx_MspInit(&SpiHandle);
    HAL_SPI_Init(&SpiHandle);
  }
}


static void SPIx_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  //Za��czamy zegar dla SPI
  SPIaz2_CLOCK_ENABLE();

  //To samo dla zegara GPIO
  SPIaz2_GPIO_CLK_ENABLE();


  //Konfigurujemy MOSI i MISO
  //Konfigurujemy SCK
  //Konfigurujemu SPIaz2
  GPIO_InitStructure.Pin = (SPIaz2_SCK_PIN | SPIaz2_MOSI_PIN | SPIaz2_MISO_PIN);
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;//pull down, w d�
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.Alternate = SPIaz2_AF;
  HAL_GPIO_Init(SPIaz2_GPIO_PORT, &GPIO_InitStructure);
}

//Dwie funkcje deinicjalizuj�ce
void SPIx_DeInit(void)
{
  if (HAL_SPI_GetState(&SpiHandle) != HAL_SPI_STATE_RESET)
  {

    HAL_SPI_DeInit(&SpiHandle);
    SPIx_MspDeInit();
  }
}


static void SPIx_MspDeInit(void)
{

  SPIaz2_GPIO_CLK_ENABLE();


  HAL_GPIO_DeInit(SPIaz2_GPIO_PORT, (SPIaz2_SCK_PIN | SPIaz2_MOSI_PIN | SPIaz2_MISO_PIN));

  SPIaz2_GPIO_FORCE_RESET();
  SPIaz2_GPIO_RELEASE_RESET();


  SPIaz2_CLOCK_DISABLE();
}


static uint8_t SPIx_WriteRead(uint8_t Byte)
{
  uint8_t receivedbyte;

  //W��czamy SPI
  __HAL_SPI_ENABLE(&SpiHandle);
  //sprawdzamy oflagowanie TXE
  while ((SpiHandle.Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);

  //wpisujemy dan�
  *((__IO uint8_t *)&SpiHandle.Instance->DR) = Byte;

  while ((SpiHandle.Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  receivedbyte = *((__IO uint8_t *)&SpiHandle.Instance->DR);

  //czekamy na flag� BSY
  while ((SpiHandle.Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY);
  while ((SpiHandle.Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);

  //wy��czamy SPI
  __HAL_SPI_DISABLE(&SpiHandle);

  return receivedbyte;
}


static void SPIx_Write(uint8_t Byte)
{
	//W��czamy SPI
  __HAL_SPI_ENABLE(&SpiHandle);
  //sprawdzamy oflagowanie TXE
  while ((SpiHandle.Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);

  //wpisujemy dan�
  *((__IO uint8_t *)&SpiHandle.Instance->DR) = Byte;

  //czekamy na flag� BSY
  while ((SpiHandle.Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);

  //wy��czamy SPI
  __HAL_SPI_DISABLE(&SpiHandle);
}
static uint8_t SPIx_Read(void)
{
  uint8_t receivedbyte;

  __HAL_SPI_ENABLE(&SpiHandle);
  __DSB();
  __DSB();
  __DSB();
  __DSB();
  __DSB();
  __DSB();
  __DSB();
  __DSB();
  __HAL_SPI_DISABLE(&SpiHandle);

  while ((SpiHandle.Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);//odczytujemy odebran� dan�

  receivedbyte = *(__IO uint8_t *)&SpiHandle.Instance->DR;


  while ((SpiHandle.Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);//musimy zaczekac na reset flagi BSY


  return receivedbyte;
}

//LINK
void ZYROS_SPI_INIT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;





  //Konfigurujemy piny do kontroli �yroskopu. W tym te� do kontroli chip
  GYRO_CS_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = GYRO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GYRO_CS_GPIO_PORT, &GPIO_InitStructure);


  GYRO_CS_HIGH();


//Konfiguracje rzeczy potrzebnych do wykrywania przerwa� (INT1, INT2, GPIO piny i zegar)
  GYRO_INT1_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = GYRO_INT1_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GYRO_INT1_GPIO_PORT, &GPIO_InitStructure);

  GYRO_INT2_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = GYRO_INT2_PIN;
  HAL_GPIO_Init(GYRO_INT2_GPIO_PORT, &GPIO_InitStructure);

  SPIx_Init();

}



void ZYROS_SPI_DEINIT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //za��czamy zegar GPIO
  GYRO_CS_GPIO_CLK_ENABLE();

  GPIO_InitStructure.Pin = GYRO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GYRO_CS_GPIO_PORT, &GPIO_InitStructure);


  GYRO_CS_HIGH();

  GYRO_INT1_GPIO_CLK_ENABLE();
  GYRO_INT2_GPIO_CLK_ENABLE();


  //deinicjalizacja pin�w do przerwa� INT1 i INT2
  HAL_GPIO_DeInit(GYRO_INT1_GPIO_PORT, GYRO_INT1_PIN);
  HAL_GPIO_DeInit(GYRO_INT2_GPIO_PORT, GYRO_INT2_PIN);


  SPIx_DeInit();
}


void ZYROS_SPI_WRITE(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{

	//Konfiguracja bit�w MS. Do czego to?
	// Gdy bit MS jest ustawiony na 0, adress b�dzie zawsze taki sam niezale�nie od liczby wielu odczytywa� i zapis�w do tego adresu
	// Gdy bit MS jest ustawiony na 1, adress b�dzie si� zmienia� poprzez wzrost warto�ci w zale�nosci od liczby wielu odczytywa� i zapis�w do tego adresu
  if (NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }

  //ustawienie chipu na niski na pocz�tku transmisji
  GYRO_CS_LOW();
  __SPI_DIRECTION_2LINES(&SpiHandle);


  //wy�lij adres do wskazanego rejestry
  SPIx_WriteRead(WriteAddr);


  //wy�lij dan�, kt�ra zostanie zapisana do urz�dzenia (akcel czy zyro). Pierwszy idzie bit MSB
  while (NumByteToWrite >= 0x01)
  {
    SPIx_WriteRead(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }

  //ustawienie chipu na wysoki na pocz�tku transmisji
  GYRO_CS_HIGH();
}


//odczytuje funkcja zbi�r danych z �yroskopu. wska�nik zapodaje je do buffera. Read Addr, to co� w rodzaju wewn�trznego adrsu �yroskopu
void ZYROS_SPI_READ(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
  if (NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  //ustawienie chipu na niski na pocz�tku transmisji
  GYRO_CS_LOW();
  __SPI_DIRECTION_2LINES(&SpiHandle);
  //wy�lij adres do wskazanego rejestry
  SPIx_WriteRead(ReadAddr);

  //wy�lij dan�, kt�ra zostanie zapisana do urz�dzenia (akcel czy zyro). Pierwszy idzie bit MSB
  while (NumByteToRead > 0x00)
  {

	  //wy�lij dummy bite aby wygenerowa� zegar dla �yroskopu (slave'a)
    *pBuffer = SPIx_WriteRead(0x00);
    NumByteToRead--;
    pBuffer++;
  }

  //ustawienie chipu na wysoki na pocz�tku transmisji
  GYRO_CS_HIGH();
}
//LINK

void AKCEL_SPI_INIT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //Konfigurujemy piny do kontroli �yroskopu. W tym te� do kontroli chip

  ACCELERO_CS_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = ACCELERO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ACCELERO_CS_GPIO_PORT, &GPIO_InitStructure);


  ACCELERO_CS_HIGH();

  SPIx_Init();
}

//dekonfiguracja akclerometru SPI
void AKCEL_SPI_DEINIT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  //za��czamy zegar GPIO
  ACCELERO_CS_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = ACCELERO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ACCELERO_CS_GPIO_PORT, &GPIO_InitStructure);


  ACCELERO_CS_HIGH();


  SPIx_DeInit();
}


void AKCEL_SPI_ITCONFIG(void)
{
}

//funkcja zapisuje zbi�r danych z �yroskopu. wska�nik zapodaje je do buffera. Read Addr, to co� w rodzaju wewn�trznego adrsu �yroskopu
void AKCEL_SPI_WRITE(uint8_t RegisterAddr, uint8_t Value)
{
  ACCELERO_CS_LOW();
  __SPI_DIRECTION_1LINE_TX(&SpiHandle);

  //wywo�anie funkcji odczytu SPI
  SPIx_Write(RegisterAddr);
  SPIx_Write(Value);
  ACCELERO_CS_HIGH();
}

//funkcja odczytuje zbi�r danych z �yroskopu. wska�nik zapodaje je do buffera. Read Addr, to co� w rodzaju wewn�trznego adrsu �yroskopu
uint8_t AKCEL_SPI_READ(uint8_t RegisterAddr)
{
  RegisterAddr = RegisterAddr | ((uint8_t)0x80);
  ACCELERO_CS_LOW();
  __SPI_DIRECTION_1LINE_TX(&SpiHandle);
  SPIx_Write(RegisterAddr);
  __SPI_DIRECTION_1LINE_RX(&SpiHandle);
  uint8_t val = SPIx_Read();
  ACCELERO_CS_HIGH();
  return val;
}
