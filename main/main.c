#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "mpu6050.h"
#include <math.h>
#include "hardware/uart.h"
#include <stdbool.h>
#include <stdint.h>



#include "Fusion.h"
#define SAMPLE_PERIOD (0.1f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

typedef struct adc {
    int axis;
    int val;
} adc_t;

QueueHandle_t xQueuePos;

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false); // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        ;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false); // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    // configuracao do I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    FusionEuler euler;
    //int yaw_anterior = 0;

    while (true) {

        mpu6050_read_raw(acceleration, gyro, &temp);
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // === Limita para evitar ruído pequeno ===
        adc_t adc_x = {.axis = 1, .val = -euler.angle.roll};
        xQueueSend(xQueuePos, &adc_x, pdMS_TO_TICKS(10));
        adc_t adc_y = {.axis = 0, .val = euler.angle.pitch};
        xQueueSend(xQueuePos, &adc_y, pdMS_TO_TICKS(10));

        
        
        // === Controle de clique baseado no Yaw ===
        if (abs(acceleration[1]) > 10000) {
            adc_t click = {.axis = 3, .val = 0}; // Pressiona
            xQueueSend(xQueuePos, &click, pdMS_TO_TICKS(10));
        }

                vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void incline_task(void *p) {
    adc_gpio_init(26);
    uint16_t samples[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t sum = 0;
    int index = 0;

    while (1) {
        adc_select_input(0); // Select ADC input 0 (GPIO26)
        uint16_t result_x = adc_read();
        sum -= samples[index];
        samples[index] = result_x;
        sum += result_x;
        index = (index + 1) % 10;
        uint16_t filtered_val = sum / 10;

        int result = ((int)filtered_val - 2047) * 255 / 2047;

        if (!((result > -200) && (result < 200))) {
            adc_t adc_z;
            adc_z.axis = 4;
            adc_z.val = result;
            xQueueSend(xQueuePos, &adc_z, pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void uart_task(void *p) {
    adc_t package;
    while (1) {
        if (xQueueReceive(xQueuePos, &package, 0)) {
            uint8_t lsb = package.val & 0xFF;
            uint8_t msb = (package.val >> 8 * 3) & 0xFF;
            uart_putc_raw(uart0, package.axis);
            uart_putc_raw(uart0, lsb);
            uart_putc_raw(uart0, msb);
            uart_putc_raw(uart0, 0xFF);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

int main() {
    stdio_init_all();
    adc_init();

    xQueuePos = xQueueCreate(16, sizeof(adc_t));
    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(incline_task, "incline_Task", 4095, NULL, 2, NULL);
    xTaskCreate(uart_task, "oled_task", 4095, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
