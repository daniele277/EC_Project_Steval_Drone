typedef struct {
	// Parametri di tuning
	float Kp;
	float Ki;
	float Kd;

	// Memoria interna
	float integral;
	float prev_error;

	// Limiti fisici
	float out_max; // Limiti attuatori
	float out_min;
	float int_max; // Anti-windup
	float int_min;

} PID;

void PID_Init(PID *pid, float kp, float ki, float kd, float out_max, float out_min);
float PID_Compute(PID *pid, float setpoint, float measured, float dt);

