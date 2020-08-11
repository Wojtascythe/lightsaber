/*
 * gyros.c
 *
 *  Created on: 30.03.2018
 *      Author: wojta
 */


//Include
#include "gyros.h"

//Private

static ZYROS_DrvTypeDef *zyrDRV;

uint8_t ZYROS_Init(void)
{
  uint8_t werdykt = Zyros_err;
  uint16_t ctrl = 0x0000;
  ZYROS_InitTypeDef l3gyro_in_struct;
  ZYROS_FilterConfigTypeDef l3gyro_filtr_struct = {0, 0};

  if ((l3gyroDRV.ReadID() == I_AM_L3GYRO) || (l3gyroDRV.ReadID() == I_AM_L3GYRO_TR))
  {

	  /// Inicjujemy strukturê do sterowania ¿yroskopem
    zyrDRV = &l3gyroDRV;

    ///Konfigurujemy ¿yroskop
    l3gyro_in_struct.Power_Mode = L3GYRO_MODE_ACTIVE;
    l3gyro_in_struct.Output_DataRate = L3GYRO_OUTPUT_DATARATE_1;
    l3gyro_in_struct.Axes_Enable = L3GYRO_AXES_ENABLE;
    l3gyro_in_struct.Band_Width = L3GYRO_BANDWIDTH_4;
    l3gyro_in_struct.BlockData_Update = L3GYRO_BlockDataUpdate_Continous;
    l3gyro_in_struct.Endianness = L3GYRO_BLE_LSB;
    l3gyro_in_struct.Full_Scale = L3GYRO_FULLSCALE_500;


    ctrl = (uint16_t)(l3gyro_in_struct.Power_Mode | l3gyro_in_struct.Output_DataRate | l3gyro_in_struct.Axes_Enable | l3gyro_in_struct.Band_Width);
   ctrl |= (uint16_t)((l3gyro_in_struct.BlockData_Update | l3gyro_in_struct.Endianness | l3gyro_in_struct.Full_Scale) << 8);
//////////
    /////Inicjalizujemy filtr, konfigurujemy go i za³¹czamy
    zyrDRV->Init(ctrl);

    l3gyro_filtr_struct.HighPassFilter_Mode_Selection = L3GYRO_HPM_NORMAL_MODE_RES;
    l3gyro_filtr_struct.HighPassFilter_CutOff_Frequency = L3GYRO_HPFCF_0;

    ctrl = (uint8_t)((l3gyro_filtr_struct.HighPassFilter_Mode_Selection | l3gyro_filtr_struct.HighPassFilter_CutOff_Frequency));


    zyrDRV->FilterConfig(ctrl) ;


    zyrDRV->FilterCmd(L3GYRO_HIGHPASSFILTER_ENABLE);

    werdykt = Zyros_ok;/// jeœli odczytaliœmy dobre id i bez komplikacji wyszed³ nam powy¿szy proces to zatwierdzamy po³¹czenie
  }
  else
  {
	  werdykt = Zyros_err;//... albo nie
  }

  return werdykt;
}

void ZYROS_DeInit(void)
{
	ZYROS_SPI_DEINIT();
}



void ZYROS_LowPower(void)////Funkcja ma ustawic ¿yroskop na stan niski
{
  uint16_t ctrl = 0x0000;
  ZYROS_InitTypeDef l3gyro_in_struct;

  /// Pozwalamy sobie na to ¿eby skonfigurowac tylko stan
  l3gyro_in_struct.Power_Mode = L3GYRO_MODE_POWERDOWN;

  ctrl = (uint16_t)(l3gyro_in_struct.Power_Mode);

  // Ustawiamy na stan niski
  zyrDRV->LowPower(ctrl);


}


uint8_t ZYROS_ReadID(void)///Funkcja ma odczytac ID ¿yroskopu
{
  uint8_t id = 0x00;

  if (zyrDRV->ReadID != NULL)
  {
    id = zyrDRV->ReadID();
  }
  return id;
}


void ZYROS_Reset(void)///Funkcja ma resetowac ¿yroskop
{
  if (zyrDRV->Reset != NULL)
  {
    zyrDRV->Reset();
  }
}


void ZYROS_ITConfig(ZYROS_InterruptConfigTypeDef *wIntConfig)// Konfiguracja przerwañ poprzez wskaŸnik wIntConfig na strukturê z danymi do przerwañ
{
  uint16_t interruptconfig = 0x0000;

  if (zyrDRV->ConfigIT != NULL)
  {

	  //Ustawiamy przerwania dla osi i latch?
    interruptconfig |= ((uint8_t)(wIntConfig->Latch_Request | wIntConfig->Interrupt_Axes) << 8);
    interruptconfig |= (uint8_t)(wIntConfig->Interrupt_ActiveEdge);

    zyrDRV->ConfigIT(interruptconfig);
  }
}


void ZYROS_EnableIT(uint8_t pinter)//Tu w³aczamy wczeœniej skonfigurowane przerwania
{
  if (zyrDRV->EnableIT != NULL)
  {
    zyrDRV->EnableIT(pinter);
  }
}


void ZYROS_DisableIT(uint8_t pinter)//Tu wy³aczamy wczeœniej skonfigurowane przerwania
{
  if (zyrDRV->DisableIT != NULL)
  {
    zyrDRV->DisableIT(pinter);
  }
}


void ZYROS_GetXYZ(float *wfTdane)//Wyci¹gamy dane o przyœpieszeniach k¹towych z ¿yroskopu za pomoc¹ wskaŸnika na tablicê danych
{
  if (zyrDRV->GetXYZ != NULL)
  {
    zyrDRV->GetXYZ(wfTdane);
  }
}
