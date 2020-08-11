/*
 * lsm.c
 *
 *  Created on: 15.04.2018
 *      Author: wojta
 */

#include "lsm.h"

AKCEL_DrvTypeDef lsakcelDRV =
{
  LSAKCEL_Init,
  LSAKCEL_DeInit,
  LSAKCEL_ReadID,
  0,
  LSAKCEL_LowPower,
  0,
  0,
  0,
  0,
  0,
  LSAKCEL_FilterConfig,
  0,
  LSAKCEL_ReadXYZ
};
void LSAKCEL_Init(uint16_t InitStruct)
{
  uint8_t ctrl = 0x00;

  //Konfiguracja podstawowego NISKIEGO interfejsu
  AKCEL_SPI_INIT();

  //zapis danej MEME do adresu CTRL_REG1
  ctrl = (uint8_t) InitStruct;
  AKCEL_SPI_WRITE(LSAKCEL_CTRL_REG1_A, ctrl);

  //zapis danej MEME do adresu CTRL_REG4
  ctrl = ((uint8_t) (InitStruct >> 8));
  AKCEL_SPI_WRITE(LSAKCEL_CTRL_REG4_A, ctrl);
}
void LSAKCEL_DeInit(void)
{
}
uint8_t LSAKCEL_ReadID(void)
{
  uint8_t ctrl = 0x00;

  //Inicjujemy NISKI interfejs do komunikacji sPI
  AKCEL_SPI_INIT();

  //WA¯NE Dla akcelerometru trzeba w³¹czyc mozliwosc komunikacji SPI, poni¿ej
  AKCEL_SPI_WRITE(LSAKCEL_CTRL_REG4_A, 0x5);

  ///WA¯NE odczytujemy adres Who I am?
  ctrl = AKCEL_SPI_READ(LSAKCEL_WHO_AM_I_ADDR);

  return ctrl;
}
void LSAKCEL_LowPower(uint16_t Mode)
{
  uint8_t ctrl = 0x00;

  //zapis danej z MEMES do CTRL_REG1
  ctrl = AKCEL_SPI_READ(LSAKCEL_CTRL_REG1_A);

  // Czyszczenie bitów ODR
  ctrl &= ~(LSAKCEL_ACC_ODR_BITPOSITION);

  // moc ustawic na niski
  ctrl |= (uint8_t)Mode;

  //zapis adresu kontrolnego CTRL_REG1A
  AKCEL_SPI_WRITE(LSAKCEL_CTRL_REG1_A, ctrl);
}
void LSAKCEL_FilterConfig(uint8_t FilterStruct)
{
  uint8_t tmpreg;


  tmpreg = FilterStruct;


  //Zapis danych do ACC w Meme na adres CTRLREG2
  AKCEL_SPI_WRITE(LSAKCEL_CTRL_REG2_A, tmpreg);
}
void LSAKCEL_ReadXYZ(int16_t* pData)
{
  int16_t pnRawData[3];
  uint8_t ctrlx[2]={0,0};
  uint8_t buffer[6];
  uint8_t i = 0;
  uint8_t sensitivity = LSAKCEL_ACC_SENSITIVITY_8G;


  //Odczyt danych przyspieszenia w odpowiednich adresach
  ctrlx[0] = AKCEL_SPI_READ(LSAKCEL_CTRL_REG4_A);
  ctrlx[1] = AKCEL_SPI_READ(LSAKCEL_CTRL_REG5_A);


  buffer[0] = AKCEL_SPI_READ(LSAKCEL_OUT_X_L_A);
  buffer[1] = AKCEL_SPI_READ(LSAKCEL_OUT_X_H_A);
  buffer[2] = AKCEL_SPI_READ(LSAKCEL_OUT_Y_L_A);
  buffer[3] = AKCEL_SPI_READ(LSAKCEL_OUT_Y_H_A);
  buffer[4] = AKCEL_SPI_READ(LSAKCEL_OUT_Z_L_A);
  buffer[5] = AKCEL_SPI_READ(LSAKCEL_OUT_Z_H_A);

  for(i=0; i<3; i++)
  {
    pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]);
  }


  switch(ctrlx[0] & LSAKCEL_ACC_FULLSCALE_8G)//Tymczasowo wykasowa³em inne opcje czu³oœci akcelerometru.
  //Zachowa³em strukturê swictha by ³atwo by³oby dodac nowe opcje w razie testów
  {

  case LSAKCEL_ACC_FULLSCALE_8G:
    sensitivity = LSAKCEL_ACC_SENSITIVITY_8G;
    break;
  }



  for(i=0; i<3; i++)
  {
    pData[i]=(pnRawData[i]);
  }
}
