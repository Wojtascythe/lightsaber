/*
 * lsm.h
 *
 *  Created on: 15.04.2018
 *      Author: wojta
 */

#ifndef LSM_H_
#define LSM_H_

#include "akcelstruct.h"


////////////////////////////Akcelerometr///////////////////
 #define LSAKCEL_WHO_AM_I_ADDR             0x0F  //Adres identyfikacji ¿yroskopu
 #define LSAKCEL_ACT_THS_A                 0x1E
 #define LSAKCEL_ACT_DUR_A                 0x1F
 #define LSAKCEL_CTRL_REG1_A               0x20  //CTRL 1
 #define LSAKCEL_CTRL_REG2_A               0x21  //CTRL 2
 #define LSAKCEL_CTRL_REG3_A               0x22  //CTRL 3
 #define LSAKCEL_CTRL_REG4_A               0x23  //CTRL 4
 #define LSAKCEL_CTRL_REG5_A               0x24  //CTRL 5
 #define LSAKCEL_CTRL_REG6_A               0x25  //CTRL 6
 #define LSAKCEL_CTRL_REG7_A               0x26  //CTRL 7
 #define LSAKCEL_STATUS_REG_A              0x27  //adres statusu
 #define LSAKCEL_OUT_X_L_A                 0x28  //adres danej x niski
 #define LSAKCEL_OUT_X_H_A                 0x29  //adres danej x wysoki
 #define LSAKCEL_OUT_Y_L_A                 0x2A  //adres danej y niski
 #define LSAKCEL_OUT_Y_H_A                 0x2B  //adres danej y wysoki
 #define LSAKCEL_OUT_Z_L_A                 0x2C  //adres danej z niski
 #define LSAKCEL_OUT_Z_H_A                 0x2D  //adres danej z wysoki
 #define LSAKCEL_FIFO_CTRL                 0x2E  //<-Upewnic sie czy potrzebne
 #define LSAKCEL_FIFO_SRC                  0x2F

 #define LSAKCEL_IG_CFG1_A                 0x30  //Adres konfiguracji pierwszego przerwania
 #define LSAKCEL_IG_SRC1_A                 0x31  //Adres pierwszego przerwania
 #define LSAKCEL_IG_THS_X1_A               0x32
 #define LSAKCEL_IG_THS_Y1_A               0x33
 #define LSAKCEL_IG_THS_Z1_A               0x34

 #define LSAKCEL_IG_DUR1_A                 0x32
 #define LSAKCEL_INT1_DURATION_A           0x33  //adres trwania przerwania 1

 #define LSAKCEL_INT2_CFG_A                0x34  //Adres konfiguracji drugiego przerwania
 #define LSAKCEL_INT2_SOURCE_A             0x35  //Adres drugiego przerwania
 #define LSAKCEL_INT2_THS_A                0x36
 #define LSAKCEL_INT2_DURATION_A           0x37  //adres trwania przerwania 2

 #define LSAKCEL_CLICK_CFG_A               0x38
 #define LSAKCEL_CLICK_SOURCE_A            0x39
 #define LSAKCEL_CLICK_THS_A               0x3A

 #define LSAKCEL_TIME_LIMIT_A              0x3B
 #define LSAKCEL_TIME_LATENCY_A            0x3C
 #define LSAKCEL_TIME_WINDOW_A             0x3D
 //Tryb pracy akcelerometru
#define LSAKCEL_ACC_ID                      ((uint8_t)0x41)
//
#define LSAKCEL_ACC_ODR_BITPOSITION         ((uint8_t)0x70)
#define LSAKCEL_ACC_ODR_OFF                 ((uint8_t)0x00)
#define LSAKCEL_ACC_ODR_10_HZ               ((uint8_t)0x10)
#define LSAKCEL_ACC_ODR_50_HZ               ((uint8_t)0x20)
#define LSAKCEL_ACC_ODR_100_HZ              ((uint8_t)0x30)
#define LSAKCEL_ACC_ODR_200_HZ              ((uint8_t)0x40)
#define LSAKCEL_ACC_ODR_400_HZ              ((uint8_t)0x50)
#define LSAKCEL_ACC_ODR_800_HZ              ((uint8_t)0x60)
//
#define LSAKCEL_ACC_X_ENABLE                ((uint8_t)0x01)
#define LSAKCEL_ACC_Y_ENABLE                ((uint8_t)0x02)
#define LSAKCEL_ACC_Z_ENABLE                ((uint8_t)0x04)
#define LSAKCEL_ACC_AXES_ENABLE             ((uint8_t)0x07)
#define LSAKCEL_ACC_AXES_DISABLE            ((uint8_t)0x00)
//
#define LSAKCEL_ACC_HR_ENABLE               ((uint8_t)0x80)
#define LSAKCEL_ACC_HR_DISABLE              ((uint8_t)0x00)
 //

//Tryb komunikacji
 #define  LSAKCEL_ACC_SPI_MODE          ((uint8_t) 0x01)


//Maksymalna skala akcelerometru
 #define LSAKCEL_ACC_FULLSCALE_8G            ((uint8_t)0x30)

//Maksymalna czu³oœc akcelerometru. Bêdzie trzeba wybrac w trakcie, ale domski mowil ¿e raczej du¿a

 #define LSAKCEL_ACC_SENSITIVITY_8G     ((uint8_t)4)
 #define LSAKCEL_ACC_SENSITIVITY_16G    ((uint8_t)12)


//Aktualizacja bloku danych akcelerometru. Albo w trybie ci¹g³ym albo pojedyncza aktualizacja po zakoñczeniu odczytywania bitów MS i LS
 #define LSAKCEL_ACC_BDU_CONTINUOUS   ((uint8_t)0x00)
 #define LSAKCEL_ACC_BDU_MSBLSB       ((uint8_t)0x08)

//Wybór kolejnosci odczytywania danych, bity rosn¹co lub malej¹co
 #define LSAKCEL_ACC_BLE_LSB                 ((uint8_t)0x00) //LSB jest nizej w adresie
 #define LSAKCEL_ACC_BLE_MSB                 ((uint8_t)0x40) //MSB jest ni¿ej w adresie


//Wybór trybu filtru górnoprzepustowego dla ackelerometru
 #define LSAKCEL_ACC_HPM_REF_SIGNAL          ((uint8_t)0x08)
 #define LSAKCEL_ACC_HPM_NORMAL_MODE         ((uint8_t)0x00)


//Wybór czêstotliwoœci filtru górnoprzepustowego
 #define LSAKCEL_ACC_DFC1_ODRDIV50       ((uint8_t)0x00)
 #define LSAKCEL_ACC_DFC1_ODRDIV100      ((uint8_t)0x20)
 #define LSAKCEL_ACC_DFC1_ODRDIV9        ((uint8_t)0x40)
 #define LSAKCEL_ACC_DFC1_ODRDIV400      ((uint8_t)0x60)

//Wybór stanu filtru
 #define LSAKCEL_ACC_HPF_DISABLE         ((uint8_t)0x00)
 #define LSAKCEL_ACC_HPF_ENABLE          ((uint8_t)0x08)

///wybór stanu klikniêcia, sprawdzic czy potrzebne!!!!!!!!!!!!
 #define LSAKCEL_ACC_HPF_CLICK_DISABLE   ((uint8_t)0x00)
 #define LSAKCEL_ACC_HPF_CLICK_ENABLE    ((uint8_t)0x04)


//HPI2S???? Sprawdzic
 #define LSAKCEL_ACC_HPI2S_INT1_DISABLE  ((uint8_t)0x00)
 #define LSAKCEL_ACC_HPI2S_INT1_ENABLE	  ((uint8_t)0x01)
 #define LSAKCEL_ACC_HPI2S_INT2_DISABLE  ((uint8_t)0x00)
 #define LSAKCEL_ACC_HPI2S_INT2_ENABLE   ((uint8_t)0x02)


//Konfiguracja przerwañ
 #define LSAKCEL_IT1_CLICK               ((uint8_t)0x80)
 #define LSAKCEL_IT1_AOI1                ((uint8_t)0x40)
 #define LSAKCEL_IT1_AOI2                ((uint8_t)0x20)
 #define LSAKCEL_IT1_DRY1                ((uint8_t)0x10)
 #define LSAKCEL_IT1_DRY2                ((uint8_t)0x08)
 #define LSAKCEL_IT1_WTM                 ((uint8_t)0x04)
 #define LSAKCEL_IT1_OVERRUN             ((uint8_t)0x02)



 #define LSAKCEL_IT2_CLICK               ((uint8_t)0x80)
 #define LSAKCEL_IT2_INT1                ((uint8_t)0x40)
 #define LSAKCEL_IT2_INT2                ((uint8_t)0x20)
 #define LSAKCEL_IT2_BOOT                ((uint8_t)0x10)
 #define LSAKCEL_IT2_ACT                 ((uint8_t)0x08)
 #define LSAKCEL_IT2_HLACTIVE            ((uint8_t)0x02)


//?????????????????????????????????????????????????????
 #define LSAKCEL_OR_COMBINATION          ((uint8_t)0x00)
 #define LSAKCEL_AND_COMBINATION         ((uint8_t)0x80)
 #define LSAKCEL_MOV_RECOGNITION         ((uint8_t)0x40)
 #define LSAKCEL_POS_RECOGNITION         ((uint8_t)0xC0)
//Adresy danych z osi
 #define LSAKCEL_Z_HIGH                  ((uint8_t)0x20)
 #define LSAKCEL_Z_LOW                   ((uint8_t)0x10)
 #define LSAKCEL_Y_HIGH                  ((uint8_t)0x08)
 #define LSAKCEL_Y_LOW                   ((uint8_t)0x04)
 #define LSAKCEL_X_HIGH                  ((uint8_t)0x02)
 #define LSAKCEL_X_LOW                   ((uint8_t)0x01)
//////////////???????????????????????
 #define LSAKCEL_Z_DOUBLE_CLICK          ((uint8_t)0x20)
 #define LSAKCEL_Z_SINGLE_CLICK          ((uint8_t)0x10)
 #define LSAKCEL_Y_DOUBLE_CLICK          ((uint8_t)0x08)
 #define LSAKCEL_Y_SINGLE_CLICK          ((uint8_t)0x04)
 #define LSAKCEL_X_DOUBLE_CLICK          ((uint8_t)0x02)
 #define LSAKCEL_X_SINGLE_CLICK          ((uint8_t)0x01)
//Stan przerwania akcelerometru
 #define LSAKCEL_INT1INTERRUPT_DISABLE   ((uint8_t)0x00)
 #define LSAKCEL_INT1INTERRUPT_ENABLE    ((uint8_t)0x80)
//
 #define LSAKCEL_INT1INTERRUPT_LOW_EDGE  ((uint8_t)0x20)
 #define LSAKCEL_INT1INTERRUPT_HIGH_EDGE ((uint8_t)0x00)



void    LSAKCEL_Init(uint16_t InitStruct);
void    LSAKCEL_DeInit(void);
uint8_t LSAKCEL_ReadID(void);
void    LSAKCEL_LowPower(uint16_t Mode);
void    LSAKCEL_FilterConfig(uint8_t FilterStruct);
void    LSAKCEL_FilterCmd(uint8_t HighPassFilterState);
void    LSAKCEL_ReadXYZ(int16_t* pData);
void    LSAKCEL_FilterClickCmd(uint8_t HighPassFilterClickState);
void    LSAKCEL_IT1Enable(uint8_t LSAKCEL_IT);
void    LSAKCEL_IT1Disable(uint8_t LSAKCEL_IT);
void    LSAKCEL_IT2Enable(uint8_t LSAKCEL_IT);
void    LSAKCEL_IT2Disable(uint8_t LSAKCEL_IT);
void    LSAKCEL_ClickITEnable(uint8_t ITClick);
void    LSAKCEL_ClickITDisable(uint8_t ITClick);
void    LSAKCEL_ZClickITConfig(void);

extern AKCEL_DrvTypeDef lsakcelDRV;

extern void    AKCEL_SPI_INIT(void);
extern void    AKCEL_SPI_ITCONFIG(void);
extern void    AKCEL_SPI_WRITE(uint8_t RegisterAddr, uint8_t Value);
extern uint8_t AKCEL_SPI_READ(uint8_t RegisterAddr);
extern void    AKCEL_SPI_DEINIT(void);
#endif /* LSM_H_ */
