/*
 * l3g.c
 *
 *  Created on: 15.04.2018
 *      Author: wojta
 */

#include "l3g.h"


ZYROS_DrvTypeDef l3gyroDRV =
{
		L3GYRO_Init,
		L3GYRO_DeInit,
		L3GYRO_ReadID,
		L3GYRO_RebootCmd,
		L3GYRO_LowPower,
		L3GYRO_INT1InterruptConfig,
		L3GYRO_EnableIT,
		L3GYRO_DisableIT,
  0,
  0,
  L3GYRO_FilterConfig,
  L3GYRO_FilterCmd,
  L3GYRO_ReadXYZAngRate
};
////inicjalizacja obiektu  l3gyro za pomoc¹ wskaŸnika InitStruct, który wskazuje na InitTypeDef, strukturê do konfiguracji  l3gyro

void L3GYRO_Init(uint16_t InitStruct)
{
  uint8_t ctrl = 0x00;

  //Konfiguracja podstawowego interfejsu
  ZYROS_SPI_INIT();

  //zapis danej MEME do adresu CTRL_REG1
  ctrl = (uint8_t) InitStruct;
  ZYROS_SPI_WRITE(&ctrl, L3GYRO_CTRL_REG1_ADDR, 1);

  //zapis danej MEME do adresu CTRL_REG4
  ctrl = (uint8_t) (InitStruct >> 8);
  ZYROS_SPI_WRITE(&ctrl, L3GYRO_CTRL_REG4_ADDR, 1);
}
void L3GYRO_DeInit(void)
{
}
//Czytanie ID obiektu ¿yroskopu  L3GYRO
uint8_t L3GYRO_ReadID(void)
{
  uint8_t tmp;

  //Inicjujemy NISKI interfejs do komunikacji sPI
  ZYROS_SPI_INIT();

  ///WA¯NE odczytujemy adres Who I am?
  ZYROS_SPI_READ(&tmp, L3GYRO_WHO_AM_I_ADDR, 1);

  //przekazuje do tymczasowgo wskaŸnika adres ID
  return (uint8_t)tmp;
}
void L3GYRO_RebootCmd(void)
{
  uint8_t tmpreg;

  //Odczyt danej z MEME CTRL_REG5
  ZYROS_SPI_READ(&tmpreg, L3GYRO_CTRL_REG5_ADDR, 1);

  //Wy³¹czanie lub w³aczanie resetu pamiêci
  tmpreg |= L3GYRO_BOOT_REBOOTMEMORY;

  //zapis danej MEME do CTRL_REG5
  ZYROS_SPI_WRITE(&tmpreg, L3GYRO_CTRL_REG5_ADDR, 1);
}

//Ustawic obiekt w stan niski
void L3GYRO_LowPower(uint16_t InitStruct)
{
  uint8_t ctrl = 0x00;

  //zapis danej z MEMES do CTRL_REG1
  ctrl = (uint8_t) InitStruct;
  ZYROS_SPI_WRITE(&ctrl, L3GYRO_CTRL_REG1_ADDR, 1);
}
//Konfiguracja przerwania dla obiektu

void L3GYRO_INT1InterruptConfig(uint16_t Int1Config)
{
  uint8_t ctrl_cfr = 0x00, ctrl3 = 0x00;

  // odczyt adresu INT1_CFG
  ZYROS_SPI_READ(&ctrl_cfr, L3GYRO_INT1_CFG_ADDR, 1);

  //odczyt danej z CTRL_REG3
  ZYROS_SPI_READ(&ctrl3, L3GYRO_CTRL_REG3_ADDR, 1);

  ctrl_cfr &= 0x80;
  ctrl_cfr |= ((uint8_t) Int1Config >> 8);

  ctrl3 &= 0xDF;
  ctrl3 |= ((uint8_t) Int1Config);

  // zapis danej do adresu INT1_CFG
  ZYROS_SPI_WRITE(&ctrl_cfr, L3GYRO_INT1_CFG_ADDR, 1);

  // odczyt adresu CTRL_REG3
  ZYROS_SPI_WRITE(&ctrl3, L3GYRO_CTRL_REG3_ADDR, 1);
}


//Za³aczenie któregoœ przerwania, gdzie Int Sel jest wybrem które [ZAWARTOŒC METDY OGARN¥Æ]
void L3GYRO_EnableIT(uint8_t IntSel)
{
  uint8_t tmpreg;

  //odczyt adresu CTRL_REG3
  ZYROS_SPI_READ(&tmpreg, L3GYRO_CTRL_REG3_ADDR, 1);

  if(IntSel == L3GYRO_INT1)
  {
    tmpreg &= 0x7F;
    tmpreg |= L3GYRO_INT1INTERRUPT_ENABLE;
  }
  else if(IntSel == L3GYRO_INT2)
  {
    tmpreg &= 0xF7;
    tmpreg |= L3GYRO_INT2INTERRUPT_ENABLE;
  }

  //zapis danej do CTRL_REG3
  ZYROS_SPI_WRITE(&tmpreg, L3GYRO_CTRL_REG3_ADDR, 1);
}

////Wy³¹czenie któregoœ przerwania, gdzie Int Sel jest wybrem które [ZAWARTOŒC METDY OGARN¥Æ]
void L3GYRO_DisableIT(uint8_t IntSel)
{
  uint8_t tmpreg;

  ///odczyt adresu CTRL_REG3
  ZYROS_SPI_READ(&tmpreg, L3GYRO_CTRL_REG3_ADDR, 1);

  if(IntSel == L3GYRO_INT1)
  {
    tmpreg &= 0x7F;
    tmpreg |= L3GYRO_INT1INTERRUPT_DISABLE;
  }
  else if(IntSel == L3GYRO_INT2)
  {
    tmpreg &= 0xF7;
    tmpreg |= L3GYRO_INT2INTERRUPT_DISABLE;
  }

  //zapis danej do CTRL_REG3
  ZYROS_SPI_WRITE(&tmpreg, L3GYRO_CTRL_REG3_ADDR, 1);
}

//Konfiguracja filtru dla obiektu
void L3GYRO_FilterConfig(uint8_t FilterStruct)
{
  uint8_t tmpreg;

  //zapis danej do CTRL_REG2
  ZYROS_SPI_READ(&tmpreg, L3GYRO_CTRL_REG2_ADDR, 1);

  tmpreg &= 0xC0;

  //Konfiguracja filtru
  tmpreg |= FilterStruct;

  //zapis danej do CTRL_REG2
  ZYROS_SPI_WRITE(&tmpreg, L3GYRO_CTRL_REG2_ADDR, 1);
}
//wy³¹czanie i w³¹czanie filtru
void L3GYRO_FilterCmd(uint8_t HighPassFilterState)
{
  uint8_t tmpreg;

  //zapis danej do CTRL_REG5
  ZYROS_SPI_READ(&tmpreg, L3GYRO_CTRL_REG5_ADDR, 1);

  tmpreg &= 0xEF;

  tmpreg |= HighPassFilterState;

  //zapis danej do CTRL_REG5
  ZYROS_SPI_WRITE(&tmpreg, L3GYRO_CTRL_REG5_ADDR, 1);
}


//Pobierz dane o stanie L3GYRO
uint8_t L3GYRO_GetDataStatus(void)
{
  uint8_t tmpreg;

  //odczyt statusu REG dla L3(...)
  ZYROS_SPI_READ(&tmpreg, L3GYRO_STATUS_REG_ADDR, 1);

  return tmpreg;
}


//Odczytuje dane k¹ta z ¿yroskopu
void L3GYRO_ReadXYZAngRate(float *wfTdane)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;

  ZYROS_SPI_READ(&tmpreg,L3GYRO_CTRL_REG4_ADDR,1);

  ZYROS_SPI_READ(tmpbuffer,L3GYRO_OUT_X_L_ADDR,6);


  //sprawdza sposób adrsowania danych, czy to Big Endian czy Little Endian
  if(!(tmpreg & L3GYRO_BLE_MSB))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }

  //Ustaw czu³oœc ¿yroskopu. Zostawi³em do wyboru, by w testach wy³onic odpowiedni
  switch(tmpreg & L3GYRO_FULLSCALE_SELECTION)
  {
  case L3GYRO_FULLSCALE_250:
    sensitivity=L3GYRO_SENSITIVITY_250DPS;
    break;

  case L3GYRO_FULLSCALE_500:
    sensitivity=L3GYRO_SENSITIVITY_500DPS;
    break;

  case L3GYRO_FULLSCALE_2000:
    sensitivity=L3GYRO_SENSITIVITY_2000DPS;
    break;
  }

  for(i=0; i<3; i++)
  {
    wfTdane[i]=(float)(RawData[i] * sensitivity);
  }
}

