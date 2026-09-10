#include "sensor_fusion.h"

volatile float roll_est  = 0.0f;
volatile float pitch_est = 0.0f;
volatile float yaw_est = 0.0f;

volatile float altitude_baro_filtered = 0.0f;
volatile float altitude_estimated = 0.0f;
volatile float v_z = 0.0f;

void Sensor_Fusion_Update(LSM6DSR_AxesRaw_t *accel_data, LSM6DSR_AxesRaw_t *gyro_data) {

    float ax = (float)accel_data->x;
    float ay = (float)accel_data->y;
    float az = (float)accel_data->z;

    // Conversione delle letture grezze in unità fisiche
    float gx = (float)gyro_data->x * GYRO_SENSITIVITY;
    float gy = (float)gyro_data->y * GYRO_SENSITIVITY;
    float gz = (float)gyro_data->z * GYRO_SENSITIVITY;

    // Calcolo degli angoli assoluti dall'accelerometro (conversione in gradi)
    float roll_acc  = atan2f(ay, az) * (180.0f / M_PI);
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);

    // Applicazione del Filtro Complementare con la convenzione dei segni adottata dai sensori
    roll_est  = ALPHA * (roll_est + gx * DT) + (1.0f - ALPHA) * roll_acc;
    pitch_est = ALPHA * (pitch_est + gy * DT) + (1.0f - ALPHA) * pitch_acc;
    yaw_est = yaw_est + gz * DT;
}

void Altitude_Fusion_Update(LSM6DSR_AxesRaw_t *accel_data, float pressure_data, float pressure_zero, uint8_t new_baro_data) {
	// Esegue solo quando il barometro ha un nuovo dato pronto
	if (new_baro_data == 1) {
	        float altitude_raw = 44330.0f * (1.0f - powf(pressure_data / pressure_zero, 0.190295f));
	        altitude_baro_filtered = (altitude_baro_filtered * (1.0f - ALPHA_BARO)) + (altitude_raw * ALPHA_BARO);
	    }

	float ax_g = (float)accel_data->x / ACC_SENSITIVITY;
	float ay_g = (float)accel_data->y / ACC_SENSITIVITY;
	float az_g = (float)accel_data->z / ACC_SENSITIVITY;

	float pitch_rad = pitch_est * DEG_TO_RAD;
	float roll_rad  = roll_est * DEG_TO_RAD;

	// Calcolo dell'accelerazione nel sistema di riferimento fisso
	float az_earth_g = (-ax_g * sinf(pitch_rad)) +
	                   (ay_g * sinf(roll_rad) * cosf(pitch_rad)) +
	                   (az_g * cosf(roll_rad) * cosf(pitch_rad));

	// Sottrae l'accelerazione di gravità per ottenere l'accelerazione verticale nel sistema di riferimento fisso
	float az_linear = (az_earth_g - 1.0f) * G_TO_MS2;

	// Integrazione dell'accelerazione sull'asse z per trovare l'altitudine
	v_z += az_linear * DT;
	altitude_estimated += v_z * DT;

	// Filtro complementare tra barometro e accelerometro
	altitude_estimated = (altitude_estimated * ALPHA_COMP) + (altitude_baro_filtered * (1.0f - ALPHA_COMP));
}

void Altitude_Fusion_Reset(void) {
    altitude_estimated = altitude_baro_filtered;
    v_z = 0.0f;
}
