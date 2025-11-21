#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <queue.h>
#include "hardware/i2c.h"
#include "tkjhat/sdk.h"
#include <inttypes.h>

#define ACCEL_DATA_X1  0x0B
#define ACCEL_DATA_X0  0x0C
#define DEFAULT_STACK_SIZE 2048 
#define MORSE_BUFFER_SIZE 256
#define I2C_PORT    i2c0
#define ICM42670_ADDR 0x69
#define REG_VIESTI      0x75 
#define REG_PWR_MGMT0   0x1F

enum state {
    STATE_INIT = 0,
    STATE_READ_INPUT,
    STATE_PRINT
};

enum state programState = STATE_READ_INPUT; 

char morseBuffer[MORSE_BUFFER_SIZE];
int morseIndex = 0; 
int spaceCount = 0;
int prevButton1 = 1;
int prevButton2 = 1;
volatile int messageReady = 0;

static void sensor_task(void *arg);
static void print_task(void *arg);

static void sensor_task(void *arg){
    (void)arg;

    prevButton1 = gpio_get(BUTTON1);
    prevButton2 = gpio_get(BUTTON2);

    for(;;){
        int b1 = gpio_get(BUTTON1);
        int b2 = gpio_get(BUTTON2);

        if (prevButton1 == 1 && b1 == 0) {
            printf("BUTTON1 pressed\n");

            uint8_t hi, lo;
            {
                uint8_t reg = ACCEL_DATA_X1;
                i2c_write_blocking(I2C_PORT, ICM42670_ADDR, &reg, 1, true);
                i2c_read_blocking(I2C_PORT, ICM42670_ADDR, &hi, 1, false);
            }
            {
                uint8_t reg = ACCEL_DATA_X0;
                i2c_write_blocking(I2C_PORT, ICM42670_ADDR, &reg, 1, true);
                i2c_read_blocking(I2C_PORT, ICM42670_ADDR, &lo, 1, false);
            }

            int16_t raw = (int16_t)((hi << 8) | lo);
            int angle = (int)raw;

            char symbol;
            if (angle >= 0) symbol = '-';
            else symbol = '.';

            if (morseIndex < MORSE_BUFFER_SIZE - 2) {
                morseBuffer[morseIndex++] = symbol;
                morseBuffer[morseIndex] = '\0';
                spaceCount = 0;
            }
        }

        if (prevButton2 == 1 && b2 == 0) {
            printf("BUTTON2 pressed\n");

            if (morseIndex < MORSE_BUFFER_SIZE - 2) {
                morseBuffer[morseIndex++] = ' ';
                morseBuffer[morseIndex] = '\0';
            }

            spaceCount++;
            if (spaceCount >= 3) {
                messageReady = 1;
            }
        }

        prevButton1 = b1;
        prevButton2 = b2;

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void print_task(void *arg){
    (void)arg;

    while(1){
        uint8_t hi, lo;
        {
            uint8_t reg = ACCEL_DATA_X1;
            i2c_write_blocking(I2C_PORT, ICM42670_ADDR, &reg, 1, true);
            i2c_read_blocking(I2C_PORT, ICM42670_ADDR, &hi, 1, false);
        }
        {
            uint8_t reg = ACCEL_DATA_X0;
            i2c_write_blocking(I2C_PORT, ICM42670_ADDR, &reg, 1, true);
            i2c_read_blocking(I2C_PORT, ICM42670_ADDR, &lo, 1, false);
        }
        int16_t raw = (int16_t)((hi << 8) | lo);
        int angle = (int)raw;

        if (messageReady) {
            printf(">>> MORSE-VIESTI: %s\n", morseBuffer);
            morseIndex = 0;
            morseBuffer[0] = '\0';
            spaceCount = 0;
            messageReady = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main() {
    stdio_init_all();
    init_hat_sdk();
    sleep_ms(300);

    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);
    gpio_pull_up(BUTTON1);

    gpio_init(BUTTON2);
    gpio_set_dir(BUTTON2, GPIO_IN);
    gpio_pull_up(BUTTON2);

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(DEFAULT_I2C_SCL_PIN);

    sleep_ms(10);
    {
        uint8_t buf[2] = { REG_PWR_MGMT0, 0x0F };
        i2c_write_blocking(I2C_PORT, ICM42670_ADDR, buf, 2, false);
    }

    TaskHandle_t hSensorTask = NULL;
    TaskHandle_t hPrintTask  = NULL;

    BaseType_t result;

    result = xTaskCreate(
                sensor_task,
                "sensor",
                DEFAULT_STACK_SIZE,
                NULL,
                2,
                &hSensorTask);

    if(result != pdPASS) {
        printf("Sensor task creation failed\n");
        return 0;
    }

    result = xTaskCreate(
                print_task,
                "print",
                DEFAULT_STACK_SIZE,
                NULL,
                2,
                &hPrintTask);

    if(result != pdPASS) {
        printf("Print task creation failed\n");
        return 0;
    }

    vTaskStartScheduler();
    return 0;
}
