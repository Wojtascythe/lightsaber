/*
 * gyrostruct.h
 *
 *  Created on: 15.04.2018
 *      Author: wojta
 */

#ifndef GYROSTRUCT_H_
#define GYROSTRUCT_H_
#include <stdint.h>
//////////////////////////Struktury
typedef struct
{
  void       (*Init)(uint16_t);
  void       (*DeInit)(void);
  uint8_t    (*ReadID)(void);
  void       (*Reset)(void);
  void       (*LowPower)(uint16_t);
  void       (*ConfigIT)(uint16_t);
  void       (*EnableIT)(uint8_t);
  void       (*DisableIT)(uint8_t);
  uint8_t    (*ITStatus)(uint16_t, uint16_t);
  void       (*ClearIT)(uint16_t, uint16_t);
  void       (*FilterConfig)(uint8_t);
  void       (*FilterCmd)(uint8_t);
  void       (*GetXYZ)(int16_t *);
}ZYROS_DrvTypeDef;

////////////////////////Konfiguracja ¿yroskopu

typedef struct
{
  uint8_t Power_Mode;                         //Stan ¿yroskopu
  uint8_t Output_DataRate;                    //datarate danych wyjœciowych
  uint8_t Axes_Enable;                        //za³aczenie osi
  uint8_t Band_Width;                         //wybór band width
  uint8_t BlockData_Update;                   //aktualizacja danych
  uint8_t Endianness;                         //kolejnoœc bajtów????
  uint8_t Full_Scale;                         // zakres wartœci minimalnych i maksymalnych
}ZYROS_InitTypeDef;

///////////////Struktura filtru górnoprzepustowego ? Mo¿e lepiej dolnoprzepustowy
typedef struct
{
  uint8_t HighPassFilter_Mode_Selection;      //Ustawiamy tryb
  uint8_t HighPassFilter_CutOff_Frequency;    //Czêstotliwoœc filtru
}ZYROS_FilterConfigTypeDef;

/////////////////////////////Konfiguracja przerwania ¿yroskopu
typedef struct
{
  uint8_t Latch_Request;                      //coœ w rodzaju rz¹dania przerwania
  uint8_t Interrupt_Axes;                     //przerwanania na osiach x y z
  uint8_t Interrupt_ActiveEdge;               //
}ZYROS_InterruptConfigTypeDef;



#endif /* GYROSTRUCT_H_ */
