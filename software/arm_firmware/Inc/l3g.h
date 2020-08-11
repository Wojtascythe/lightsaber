/*
 * l3g.h
 *
 *  Created on: 15.04.2018
 *      Author: wojta
 */

#ifndef L3G_H_
#define L3G_H_
#include "gyrostruct.h"


////////////////////ADRESOWANIE///////////////////////////////
 ////////////////¯yroskop////////////////////
#define L3GYRO_WHO_AM_I_ADDR          0x0F  //Adres identyfikacji ¿yroskopu
#define L3GYRO_CTRL_REG1_ADDR         0x20  //CTRL 1
#define L3GYRO_CTRL_REG2_ADDR         0x21  //CTRL 2
#define L3GYRO_CTRL_REG3_ADDR         0x22  //CTRL 3
#define L3GYRO_CTRL_REG4_ADDR         0x23  //CTRL 4
#define L3GYRO_CTRL_REG5_ADDR         0x24  //CTRL 5
#define L3GYRO_REFERENCE_REG_ADDR     0x25  //adres referencji
#define L3GYRO_OUT_TEMP_ADDR          0x26  //
#define L3GYRO_STATUS_REG_ADDR        0x27  //adres stanu
#define L3GYRO_OUT_X_L_ADDR           0x28  //adress danej x niski
#define L3GYRO_OUT_X_H_ADDR           0x29  //adress danej x wysoki
#define L3GYRO_OUT_Y_L_ADDR           0x2A  ///adress danej y niski
#define L3GYRO_OUT_Y_H_ADDR           0x2B  //adress danej y wysoki
#define L3GYRO_OUT_Z_L_ADDR           0x2C  //adress danej z niski
#define L3GYRO_OUT_Z_H_ADDR           0x2D  //adress danej z wysoki
//#define L3GYRO_FIFO_CTRL_REG_ADDR     0x2E  //<-Przeanalizowac czy to potrzebne
//#define L3GYRO_FIFO_SRC_REG_ADDR      0x2F

#define L3GYRO_INT1_CFG_ADDR          0x30  //Adres konfiguracji pierwszego przerwania
#define L3GYRO_INT1_SRC_ADDR          0x31  //Adres pierwszego przerwania
#define L3GYRO_INT1_TSH_XH_ADDR       0x32  //Threshold?
#define L3GYRO_INT1_TSH_XL_ADDR       0x33
#define L3GYRO_INT1_TSH_YH_ADDR       0x34
#define L3GYRO_INT1_TSH_YL_ADDR       0x35
#define L3GYRO_INT1_TSH_ZH_ADDR       0x36
#define L3GYRO_INT1_TSH_ZL_ADDR       0x37
#define L3GYRO_INT1_DURATION_ADDR     0x38  //adres czasu trwania pierwszego przerwania

/////////////Definicje dla ¿yroskopu/////////////////////////////////////
#define I_AM_L3GYRO                 ((uint8_t)0xD4)//Dlaczego dwie wersje, przeanalizowac
#define I_AM_L3GYRO_TR              ((uint8_t)0xD5)


 //Tryb pracy ¿yroskopu
#define L3GYRO_MODE_POWERDOWN       ((uint8_t)0x00)
#define L3GYRO_MODE_ACTIVE          ((uint8_t)0x08)



//Tryby ratingu danych wyjœciowych
#define L3GYRO_OUTPUT_DATARATE_1    ((uint8_t)0x00)
#define L3GYRO_OUTPUT_DATARATE_2    ((uint8_t)0x40)
#define L3GYRO_OUTPUT_DATARATE_3    ((uint8_t)0x80)
#define L3GYRO_OUTPUT_DATARATE_4    ((uint8_t)0xC0)
//Wybór osi
#define L3GYRO_X_ENABLE            ((uint8_t)0x02)
#define L3GYRO_Y_ENABLE            ((uint8_t)0x01)
#define L3GYRO_Z_ENABLE            ((uint8_t)0x04)
#define L3GYRO_AXES_ENABLE         ((uint8_t)0x07)
#define L3GYRO_AXES_DISABLE        ((uint8_t)0x00)
//Wybór Band Width
#define L3GYRO_BANDWIDTH_1         ((uint8_t)0x00)
#define L3GYRO_BANDWIDTH_2         ((uint8_t)0x10)
#define L3GYRO_BANDWIDTH_3         ((uint8_t)0x20)
#define L3GYRO_BANDWIDTH_4         ((uint8_t)0x30)
//Wybór skali
#define L3GYRO_FULLSCALE_250       ((uint8_t)0x00)
#define L3GYRO_FULLSCALE_500       ((uint8_t)0x10)
#define L3GYRO_FULLSCALE_2000      ((uint8_t)0x20)
#define L3GYRO_FULLSCALE_SELECTION ((uint8_t)0x30)
//Wybór czu³oœci ¿yroskopu Przeliczone z datasheetu
#define L3GYRO_SENSITIVITY_250DPS  ((float)8.75f)
#define L3GYRO_SENSITIVITY_500DPS  ((float)17.50f)
#define L3GYRO_SENSITIVITY_2000DPS ((float)70.00f)
//Aktualizacja zestawu danych
#define L3GYRO_BlockDataUpdate_Continous   ((uint8_t)0x00)
#define L3GYRO_BlockDataUpdate_Single      ((uint8_t)0x80)
//Wybór kolejnosci danych, rosn¹co malej¹co
#define L3GYRO_BLE_LSB                     ((uint8_t)0x00)//LSB jest ni¿ej w adresie
#define L3GYRO_BLE_MSB	                   ((uint8_t)0x40)//MSB jest nizej w adresie
//Wybór stanu filtru
#define L3GYRO_HIGHPASSFILTER_DISABLE      ((uint8_t)0x00)
#define L3GYRO_HIGHPASSFILTER_ENABLE	     ((uint8_t)0x10)
//Wybór punktu przerwania
#define L3GYRO_INT1                        ((uint8_t)0x00)
#define L3GYRO_INT2                        ((uint8_t)0x01)
//Wybór stanu przerwania numer 1
#define L3GYRO_INT1INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GYRO_INT1INTERRUPT_ENABLE        ((uint8_t)0x80)
//Wybór stanu przerwania numer 2
#define L3GYRO_INT2INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GYRO_INT2INTERRUPT_ENABLE        ((uint8_t)0x08)
//
#define L3GYRO_INT1INTERRUPT_LOW_EDGE      ((uint8_t)0x20)
#define L3GYRO_INT1INTERRUPT_HIGH_EDGE     ((uint8_t)0x00)
//Wybór trybu pracy bootowania
#define L3GYRO_BOOT_NORMALMODE             ((uint8_t)0x00)
#define L3GYRO_BOOT_REBOOTMEMORY           ((uint8_t)0x80)
//Wybór trybu pracy filtru
#define L3GYRO_HPM_NORMAL_MODE_RES         ((uint8_t)0x00)
#define L3GYRO_HPM_REF_SIGNAL              ((uint8_t)0x10)
#define L3GYRO_HPM_NORMAL_MODE             ((uint8_t)0x20)
#define L3GYRO_HPM_AUTORESET_INT           ((uint8_t)0x30)
//Wybór czêstotliwoœci filtru
#define L3GYRO_HPFCF_0              0x00
#define L3GYRO_HPFCF_1              0x01
#define L3GYRO_HPFCF_2              0x02
#define L3GYRO_HPFCF_3              0x03
#define L3GYRO_HPFCF_4              0x04
#define L3GYRO_HPFCF_5              0x05
#define L3GYRO_HPFCF_6              0x06
#define L3GYRO_HPFCF_7              0x07
#define L3GYRO_HPFCF_8              0x08
#define L3GYRO_HPFCF_9              0x09




//Funkcje do komunikacji
void    ZYROS_SPI_INIT(void);
void    ZYROS_SPI_DEINIT(void);
void    ZYROS_SPI_WRITE(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void    ZYROS_SPI_READ(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
//Funkcje konfiguracji ¿yroskopu
void    L3GYRO_Init(uint16_t InitStruct);
void    L3GYRO_DeInit(void);
void    L3GYRO_LowPower(uint16_t InitStruct);
uint8_t L3GYRO_ReadID(void);
void    L3GYRO_RebootCmd(void);

//Funkcje konfiguracji przerwañ
void    L3GYRO_INT1InterruptConfig(uint16_t Int1Config);
void    L3GYRO_EnableIT(uint8_t IntSel);
void    L3GYRO_DisableIT(uint8_t IntSel);

//Funkcje konfiguracji filtru
void    L3GYRO_FilterConfig(uint8_t FilterStruct);
void    L3GYRO_FilterCmd(uint8_t HighPassFilterState);
void    L3GYRO_ReadXYZAngRate(float *wfTdane);
uint8_t L3GYRO_GetDataStatus(void);
extern ZYROS_DrvTypeDef l3gyroDRV;

#endif /* L3G_H_ */
