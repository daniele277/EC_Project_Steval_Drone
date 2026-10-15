typedef struct {
	// Parametri di tuning
	float Kp;
	float Ki;
	float Kd;

	// Memoria interna
	float integral;

	// Limiti fisici
	float out_max; // Limiti attuatori
	float out_min;
	float int_max; // Limiti integrazione
	float int_min;

} PID;

void PID_Init(PID *pid, float kp, float ki, float kd, float out_max, float out_min);
void PID_Reset(PID *pid);
float PID_Compute(PID *pid, float setpoint, float measured, float rate, float dt);

