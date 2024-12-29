/*
 * PID.h
 *
 *  Created on: Dec 24, 2024
 *      Author: burakguzeller
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
#include <stdio.h>
#include <string.h>

typedef struct {
	float Kp;			   /*!< Proportional parameter  		*/
	float Ki;			   /*!< İntegral parameter 				*/
	float Kd;   		   /*!< Derivative parameter 			*/
	float Ts;			   /*!< zaman damgasi					*/
	float Totalerror;	   /*!< toplam hata degeri				*/
	float lastError;	   /*!< son hata degeri					*/
	float output;		   /*!< pid cikis degeri			    */
	float outputMin;
	float outputMax;

}PID_Typedef_t;

void PID_Init(PID_Typedef_t *pid, float Kp, float Ki, float Kd, float Ts, float outMax, float outMin);
void PID_ChangeParam(PID_Typedef_t *pid, float Kp, float Ki, float Kd);
void PID_Control(PID_Typedef_t *pid, float setValue, float measuredValue);
float PID_Output(PID_Typedef_t *pid);

#endif /* INC_PID_H_ */
