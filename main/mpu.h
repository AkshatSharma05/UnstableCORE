#pragma once
#include "mpu6050.h"
#include "esp_log.h"

extern mpu6050_handle_t mpu6050;

extern float gx_bias, gy_bias, gz_bias;

extern mpu6050_acce_value_t acce;  //angular acc
extern mpu6050_gyro_value_t gyro;  //angular vel
extern complimentary_angle_t filt;

void mpu_init(i2c_port_t port);