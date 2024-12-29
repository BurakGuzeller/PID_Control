/*
 * PID.c
 *
 *  Created on: Dec 24, 2024
 *      Author: burakguzeller
 */

#include "PID.h"

void PID_Init(PID_Typedef_t *pid, float Kp, float Ki, float Kd, float Ts, float outMax, float outMin) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->Ts = Ts;
	pid->lastError = 0;
	pid->Totalerror = 0;
	pid->output = 0;
	pid->outputMin = outMin;
	pid->outputMax = outMax;

}

void PID_ChangeParam(PID_Typedef_t *pid, float Kp, float Ki, float Kd) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
}

void PID_Control(PID_Typedef_t *pid, float setValue, float measuredValue) {
	if (pid->Ts <= 0) {
	    return;
	}

	float error = setValue - measuredValue; // cikan hata
	pid->Totalerror += error * pid->Ts;

	// proportian calculate
	float P = pid->Kp * error;

	// Integral calculate
	float I = pid->Ki * pid->Totalerror;

	// Derivative calculate
    float D = pid->Kd * (error - pid->lastError) / pid->Ts;

    pid->output = P + I + D;
	pid->lastError = error;

	if(pid->output < pid->outputMin) {
	    pid->output = pid->outputMin;
	    pid->Totalerror -= error * pid->Ts; // Integral birikmesini önlemek için geri al.
	}
	else if (pid->output > pid->outputMax) {
	    pid->output = pid->outputMax;
	    pid->Totalerror -= error * pid->Ts; // Integral birikmesini önlemek için geri al.
	}
}

float PID_Output(PID_Typedef_t *pid) {
	return pid->output;
}
