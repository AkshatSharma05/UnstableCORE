#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "unity.h"
#include "mpu.h"

#define I2C_MASTER_SCL_IO 22      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000 /*!< I2C master clock frequency */

static const char *TAG = "firm_prac";


static void configure( void ) {
    //I2C INIT
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    // esp_err_t i2c_param_config(i2c_port_t i2c_num, const i2c_config_t *i2c_conf);
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));

    mpu_init(I2C_MASTER_NUM);
}

void app_main(void)
{
    configure();
    while(1) {
        mpu6050_get_gyro(mpu6050, &gyro);
        mpu6050_get_acce(mpu6050, &acce);

        gyro.gyro_x -= gx_bias;
        gyro.gyro_y -= gy_bias;
        gyro.gyro_z -= gz_bias;
        mpu6050_complimentory_filter(mpu6050, &acce, &gyro, &filt);

        // ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f, ", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
        // ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n ", acce.acce_x, acce.acce_x, acce.acce_z);
        ESP_LOGI(TAG, "roll:%.2f, pitch:%.2f\n ", filt.roll, filt.pitch);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    } 
}
