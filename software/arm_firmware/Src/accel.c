/*
 * accel.c
 *
 *  Created on: 31.03.2018
 *      Author: wojta
 */

//Include
#include "accel.h"
#include <math.h>
//#include "lsm.h"
//#include "stm32l476g_discovery.h"
static AKCEL_DrvTypeDef *akcDRV;

//Inicjalizacja akcelerometru
AKCEL_StatusTypeDef AKCEL_Init(void)
{
  AKCEL_StatusTypeDef ret = Akcel_ok;
  uint16_t ctrl = 0x0000;
  AKCEL_InitTypeDef lsakcel_InitStructure;
  AKCEL_FilterConfigTypeDef lsakcel_FilterStructure;


  if (lsakcelDRV.ReadID() != LSAKCEL_ACC_ID)
  {
    ret = Akcel_err;
  }
  else
  {
   //inicjalizacja strutkury do sterowania akceleroetetrem
    akcDRV = &lsakcelDRV;

//Konfigurujemy akcelerometr, wype�niamy jego strutkure�
    lsakcel_InitStructure.AccOutput_DataRate = LSAKCEL_ACC_ODR_400_HZ;
    lsakcel_InitStructure.Axes_Enable = LSAKCEL_ACC_AXES_ENABLE;
    lsakcel_InitStructure.AccFull_Scale = LSAKCEL_ACC_FULLSCALE_8G;
    lsakcel_InitStructure.BlockData_Update = LSAKCEL_ACC_BDU_CONTINUOUS;
    lsakcel_InitStructure.High_Resolution = LSAKCEL_ACC_HR_DISABLE;
    lsakcel_InitStructure.Communication_Mode = LSAKCEL_ACC_SPI_MODE;

    //Konfigurujemy g��wne dane akcelerometru
    ctrl = (lsakcel_InitStructure.High_Resolution | lsakcel_InitStructure.AccOutput_DataRate |  lsakcel_InitStructure.Axes_Enable | lsakcel_InitStructure.BlockData_Update);

    ctrl |= (lsakcel_InitStructure.AccFull_Scale | lsakcel_InitStructure.Communication_Mode) << 8;


    akcDRV->Init(ctrl);

    //Konfigurujemy filtr
    lsakcel_FilterStructure.HighPassFilter_Mode_Selection = LSAKCEL_ACC_HPM_NORMAL_MODE;
    lsakcel_FilterStructure.HighPassFilter_CutOff_Frequency = LSAKCEL_ACC_DFC1_ODRDIV50;
    lsakcel_FilterStructure.HighPassFilter_Stat = LSAKCEL_ACC_HPI2S_INT1_DISABLE | LSAKCEL_ACC_HPI2S_INT2_DISABLE;


    ctrl = (uint8_t)(lsakcel_FilterStructure.HighPassFilter_Mode_Selection | lsakcel_FilterStructure.HighPassFilter_CutOff_Frequency | lsakcel_FilterStructure.HighPassFilter_Stat);


    akcDRV->FilterConfig(ctrl);
  }



  return ret;
}

//Wy��czamy akcelerometr
void AKCEL_DeInit(void)
{
	AKCEL_SPI_DEINIT();
}

//Prze��czenie akcelermetru na stan niski
void AKCEL_LowPower(void)
{

  if (akcDRV != NULL)
  {
    if (akcDRV->LowPower != NULL)
    {
      akcDRV->LowPower(LSAKCEL_ACC_ODR_OFF);
    }
  }

}

//Pobranie danych z osi akcelerometru
void AKCEL_GetXYZ(int16_t *wDaneXYZ)
{
  if (akcDRV != NULL)
  {
    if (akcDRV->GetXYZ != NULL)
    {
      akcDRV->GetXYZ(wDaneXYZ);
    }
  }
}

