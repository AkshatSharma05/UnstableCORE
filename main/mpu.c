#include "mpu.h"

mpu6050_handle_t mpu6050 = NULL;

static const char *TAG = "MPU";

float gx_bias = 0, gy_bias = 0, gz_bias = 0;

mpu6050_acce_value_t acce;  //angular acc
mpu6050_gyro_value_t gyro;  //angular vel
complimentary_angle_t filt;

void mpu_init(i2c_port_t port){
    mpu6050 = mpu6050_create(port, MPU6050_I2C_ADDRESS);
    if (mpu6050 == NULL) {
        ESP_LOGE(TAG, "MPU6050 create returned NULL");
        return;
    }

    ESP_ERROR_CHECK(mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS));
    ESP_ERROR_CHECK(mpu6050_wake_up(mpu6050));

    int N = 2000;

    for (int i = 0; i < N; i++) {
        mpu6050_get_gyro(mpu6050, &gyro);
        gx_bias += gyro.gyro_x;
        gy_bias += gyro.gyro_y;
        gz_bias += gyro.gyro_z;
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
    gx_bias /= N;
    gy_bias /= N;
    gz_bias /= N;
    ESP_LOGI(TAG, "Gyro bias: %.3f %.3f %.3f", gx_bias, gy_bias, gz_bias);
}