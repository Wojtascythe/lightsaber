/*
 * accel.h
 *
 *  Created on: 31.03.2018
 *      Author: wojta
 */

#ifndef ACCEL_H_
#define ACCEL_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "komun.h"
#include "lsm.h"






 typedef enum
 {
	  Akcel_ok = 0,
	  Akcel_err = 1,
	  Akcel_t_out = 2
 }
 AKCEL_StatusTypeDef;

 AKCEL_StatusTypeDef  	 AKCEL_Init(void);
 void                    AKCEL_DeInit(void);
 void                    AKCEL_LowPower(void);
 void                    AKCEL_GetXYZ(int16_t *wDaneXYZ);





#ifdef __cplusplus
}
#endif


#endif /* ACCEL_H_ */
