/*
 * akcelstruct.h
 *
 *  Created on: 15.04.2018
 *      Author: wojta
 */

#ifndef AKCELSTRUCT_H_
#define AKCELSTRUCT_H_
#include <stdint.h>
typedef struct
{
  void      (*Init)(uint16_t);
  void      (*DeInit)(void);
  uint8_t   (*ReadID)(void);
  void      (*Reset)(void);
  void      (*LowPower)(uint16_t);
  void      (*ConfigIT)(void);
  void      (*EnableIT)(uint8_t);
  void      (*DisableIT)(uint8_t);
  uint8_t   (*ITStatus)(uint16_t);
  void      (*ClearIT)(void);
  void      (*FilterConfig)(uint8_t);
  void      (*FilterCmd)(uint8_t);
  void      (*GetXYZ)(int16_t *);
}AKCEL_DrvTypeDef;



typedef struct
{
  uint8_t Power_Mode;
  uint8_t AccOutput_DataRate;
  uint8_t Axes_Enable;
  uint8_t High_Resolution;
  uint8_t BlockData_Update;
  uint8_t Endianness;
  uint8_t AccFull_Scale;
  uint8_t Communication_Mode;
}AKCEL_InitTypeDef;


typedef struct
{
  uint8_t HighPassFilter_Mode_Selection;
  uint8_t HighPassFilter_CutOff_Frequency;
  uint8_t HighPassFilter_AOI1;
  uint8_t HighPassFilter_AOI2;
  uint8_t HighPassFilter_Data_Sel;
  uint8_t HighPassFilter_Stat;
}AKCEL_FilterConfigTypeDef;

#endif /* AKCELSTRUCT_H_ */
