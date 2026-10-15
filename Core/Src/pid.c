#include "pid.h"

void PID_Init(PID *pid, float kp, float ki, float kd, float out_max, float out_min) {
	// Inizializzazione parametri
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;

	// Inizializzazione stato
	pid->integral = 0.0f;

	// Limiti fisici
	pid->out_max = out_max;
	pid->out_min = out_min;

	// Settiamo i limiti dell'integrale ad una frazione del limite fisico in modo tale da
	// lasciare spazio all'azione proporzionale e derivativa in caso di saturazione dell'integrale
	pid->int_max = 0.35f * out_max;
	pid->int_min = -pid->int_max;
}

void PID_Reset(PID *pid) {
	pid->integral = 0.0f;
}

float PID_Compute(PID *pid, float setpoint, float measured, float rate, float dt) {
	float error;
	float p_out, i_out, d_out, tot;

	// Errore come differenza tra setpoint e valore misurato
	error = setpoint - measured;

	// Componente proporzionale
	p_out = pid->Kp * error;

	// Componente integrale
	pid->integral += error * dt;
	i_out = pid->Ki * pid->integral;

	if (i_out > pid->int_max) {
		i_out = pid->int_max;
		pid->integral = i_out / pid->Ki;
	}
	else if (i_out < pid->int_min) {
		i_out = pid->int_min;
		pid->integral = i_out / pid->Ki;
	}

	// Componente derivativa
    // D sulla misura: rate = d(measured)/dt, quindi il segno è negativo
    d_out = -pid->Kd * rate;

	// Azione di controllo
	tot = p_out + i_out + d_out;

	// Anti Wind-Up
	if (tot > pid->out_max) {
	    tot = pid->out_max;
	    if (error > 0.0f) {
	        pid->integral -= error * dt;
	    }
	} else if (tot < pid->out_min) {
	    tot = pid->out_min;
	    if (error < 0.0f) {
	        pid->integral -= error * dt;
	    }
	}

	return tot;
}
