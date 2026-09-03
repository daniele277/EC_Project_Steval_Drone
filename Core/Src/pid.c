#include "pid.h"

void PID_Init(PID *pid, float kp, float ki, float kd, float out_max, float out_min) {
	// Inizializzazione parametri
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;

	// Inizializzazione stato
	pid->integral = 0.0f;
	pid->prev_error = 0.0f;

	// Limiti fisici
	pid->out_max = out_max;
	pid->out_min = out_min;

	// Settiamo i limiti dell'integrale ad una frazione del limite fisico in modo tale da
	// lasciare spazio all'azione proporzionale e derivativa in caso di saturazione dell'integrale
	pid->int_max = 0.7f * out_max;
	pid->int_min = -pid->int_max;
}

float PID_Compute(PID *pid, float setpoint, float measured, float dt) {
	float error;
	float p_out, i_out, d_out, tot;
	float derivative;

	// Errore come differenza tra setpoint e valore misurato
	error = setpoint - measured;

	// Componente proporzionale
	p_out = pid->Kp * error;

	// Componente integrale
	pid->integral += error * dt;
	i_out = pid->Ki * pid->integral;

	// Saturazione dell'integrale (Anti-Windup)
	if (i_out > pid->int_max) {
		i_out = pid->int_max;
		if (pid->Ki != 0.0f) {
			pid->integral = i_out / pid->Ki;
		}
	} else if (i_out < pid->int_min) {
		i_out = pid->int_min;
		if (pid->Ki != 0.0f) {
			pid->integral = i_out / pid->Ki;
		}
	}

	// Componente derivativa
	// N.B.: Per il rischio di derivative kick potrebbe essere necessario usare direttamente il valore
	// di velocità angolare misurata dal giroscopio es. Kd * omega
	derivative = (error - pid->prev_error) / dt;
	d_out = pid->Kd * derivative;

	// Azione di controllo totale
	tot = p_out + i_out + d_out;

	// Saturazione dell'uscita totale (Limita l'azione sui motori)
	if (tot > pid->out_max) {
		tot = pid->out_max;
	} else if (tot < pid->out_min) {
		tot = pid->out_min;
	}

	// Salvataggio errore
	pid->prev_error = error;

	return tot;
}
