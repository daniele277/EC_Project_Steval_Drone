#include "sensor_fusion.h"

volatile float roll_est  = 0.0f;
volatile float pitch_est = 0.0f;
volatile float yaw_est = 0.0f;

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
