#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <queue.h>
#include "hardware/i2c.h"
#include "tkjhat/sdk.h"
#include <inttypes.h>

// IMU-anturipiirin rekisterit (X-akselin data)
#define ACCEL_DATA_X1  0x0B
#define ACCEL_DATA_X0  0x0C

// FreeRTOS-tehtävien pinon koko
#define DEFAULT_STACK_SIZE 2048 

// Morse-viestin maksimipituus
#define MORSE_BUFFER_SIZE 256

// I2C-väylän portti ja IMU:n osoite
#define I2C_PORT        i2c0
#define ICM42670_ADDR   0x69

// IMU:n ohjausrekisteri
#define REG_VIESTI      0x75 
#define REG_PWR_MGMT0   0x1F

// Tilakone
enum state {
    STATE_INIT = 0,
    STATE_READ_INPUT,
    STATE_PRINT
};

enum state programState = STATE_READ_INPUT; 

// Globaali Morse-puskuri ja siihen liittyvät muuttujat
char morseBuffer[MORSE_BUFFER_SIZE];
int morseIndex = 0; 
int spaceCount = 0;

// Nappien edellinen tila (pull-up: 1 = ylhäällä, 0 = painettu)
int prevButton1 = 1;
int prevButton2 = 1;

// Lippu, joka kertoo että viesti on valmis tulostettavaksi
volatile int messageReady = 0;

// Tehtäväfunktioiden etukäteismäärittelyt
static void sensor_task(void *arg);
static void print_task(void *arg);

// Tehtävä, joka lukee nappien tilan ja IMU-anturin
static void sensor_task(void *arg){
    (void)arg;

    // Alkuarvot napeille
    prevButton1 = gpio_get(BUTTON1);
    prevButton2 = gpio_get(BUTTON2);

    for(;;){
        int b1 = gpio_get(BUTTON1);
        int b2 = gpio_get(BUTTON2);

        // Reunan tunnistus: nappi 1 nousee 1 -> 0 (uusi painallus)
        if (prevButton1 == 1 && b1 == 0) {
            printf("BUTTON1 pressed\n");

            // Luetaan X-akseli kahdessa osassa (hi ja lo)
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

            // Yhdistetään 16-bittiseksi arvoksi ja tulkitaan kulmaksi
            int16_t raw = (int16_t)((hi << 8) | lo);
            int angle = (int)raw;

            // Yksinkertainen kynnys: angle >= 0 -> '-', muuten '.'
            char symbol;
            if (angle >= 0) symbol = '-';
            else symbol = '.';

            // Lisätään symboli puskuriin, jos tilaa on
            if (morseIndex < MORSE_BUFFER_SIZE - 2) {
                morseBuffer[morseIndex++] = symbol;
                morseBuffer[morseIndex] = '\0';
                spaceCount = 0;  // uutta symbolia -> nollataan välilyöntilaskuri
            }
        }

        // Reunan tunnistus: nappi 2 1 -> 0
        if (prevButton2 == 1 && b2 == 0) {
            printf("BUTTON2 pressed\n");

            // Lisätään välilyönti morse-viestiin (symbolien väli)
            if (morseIndex < MORSE_BUFFER_SIZE - 2) {
                morseBuffer[morseIndex++] = ' ';
                morseBuffer[morseIndex] = '\0';
            }

            // Kolme välilyöntiä peräkkäin = viesti valmis
            spaceCount++;
            if (spaceCount >= 3) {
                messageReady = 1;
            }
        }

        // Päivitetään edelliset tilat seuraavaa kierrosta varten
        prevButton1 = b1;
        prevButton2 = b2;

        // Pieni viive, ettei lueta liian usein
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Tehtävä, joka tulostaa valmiin Morse-viestin työaseman terminaaliin
static void print_task(void *arg){
    (void)arg;

    while(1){
        // Luetaan IMU:lta sama X-akseli (tällä hetkellä kulmaa ei käytetä tässä tehtävässä)
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
        (void)angle; // estetään "unused variable" -varoitus, jos kulmaa ei käytetä

        // Jos viesti on valmis, tulostetaan se ja nollataan tila
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
    // Sarjaportti / USB-printf käyttöön
    stdio_init_all();

    // HAT-kirjasto alustetaan (mm. nappi- ja I2C-määrittelyt)
    init_hat_sdk();
    sleep_ms(300);

    // Nappi 1: sisääntulo + pull-up
    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);
    gpio_pull_up(BUTTON1);

    // Nappi 2: sisääntulo + pull-up
    gpio_init(BUTTON2);
    gpio_set_dir(BUTTON2, GPIO_IN);
    gpio_pull_up(BUTTON2);

    // I2C-väylän alustus IMU:ta varten
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(DEFAULT_I2C_SCL_PIN);

    // IMU päälle: asetetaan virranhallintarekisteri
    sleep_ms(10);
    {
        uint8_t buf[2] = { REG_PWR_MGMT0, 0x0F };
        i2c_write_blocking(I2C_PORT, ICM42670_ADDR, buf, 2, false);
    }

    // Luodaan kaksi tehtävää: sensor_task ja print_task
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

    // Käynnistetään FreeRTOS-ajastin (ei palauta)
    vTaskStartScheduler();
    return 0;
}
