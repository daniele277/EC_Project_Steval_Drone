#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "lsm6dsr.h" // Per la struct LSM6DSR_AxesRaw_t
#include <math.h>    // Per M_PI e le funzioni matematiche

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Periodo di campionamento IMU (1 / 416 Hz)
#define DT 0.0024f

// Sensibilità giroscopio per FS di +/- 2000 dps (0.07 dps/LSB)
#define GYRO_SENSITIVITY 0.070f

// Parametro del filtro complementare
#define ALPHA 0.98f

// Costanti per sensor fusion barometro
#define ALPHA_BARO 0.02f
#define ALPHA_COMP 0.2f
#define ACC_SENSITIVITY 8196.7f
#define G_TO_MS2 9.81f
#define DEG_TO_RAD 0.0174533f

// Esportazione delle variabili globali in modo che siano visibili anche nel main
extern volatile float roll_est;
extern volatile float pitch_est;
extern volatile float yaw_est;

extern volatile float altitude_baro_filtered;
extern volatile float altitude_estimated;
extern volatile float v_z;

void Sensor_Fusion_Update(LSM6DSR_AxesRaw_t *accel_data, LSM6DSR_AxesRaw_t *gyro_data);
void Altitude_Fusion_Update(LSM6DSR_AxesRaw_t *accel_data, float pressure_data, float pressure_zero, uint8_t new_baro_data);
void Altitude_Fusion_Reset(void);

#endif /* SENSOR_FUSION_H */
