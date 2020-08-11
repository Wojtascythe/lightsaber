/*
 * gyros.h
 *
 *  Created on: 30.03.2018
 *      Author: wojta
 */

#ifndef GYROS_H_
#define GYROS_H_

#ifdef __cplusplus
extern "C" {
#endif

//includes
#include "komun.h"
#include "l3g.h"
///////Enum stanu ¿yroskopu
typedef enum
{
  Zyros_ok = 0,
  Zyros_err = 1,
  Zyros_t_out = 2
}
ZYROS_StatusTypeDef;



//Funkcje do konfiguracji ¿yroskopu
uint8_t ZYROS_Init(void);
void    ZYROS_DeInit(void);
void    ZYROS_LowPower(void);
void    ZYROS_Reset(void);
uint8_t ZYROS_ReadID(void);
void    ZYROS_ITConfig(ZYROS_InterruptConfigTypeDef *wIntConfigStruct);
void    ZYROS_EnableIT(uint8_t pinter);
void    ZYROS_DisableIT(uint8_t pinter);
void    ZYROS_GetXYZ(float *wfTdane);
#endif /* GYROS_H_ */
