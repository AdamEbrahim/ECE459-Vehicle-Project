#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "esp_err.h"
#include <string.h>

//HC-SR04 ultrasonic sensor library
#include <ultrasonic.h>

#include <driver/i2c.h>

#define LED_PIN 13
#define UT_TRIGGER_PIN 26
#define UT_ECHO_PIN 25
#define JETSON_INPUT 33

#define ESP32_I2C_PIN_SDA 23
#define ESP32_I2C_PIN_SCL 22
#define I2C_MASTER_FREQ_HZ 400000
#define JETSON_SLAVE_ADDRESS 1
#define ESP_SLAVE_ADDRESS 0x48
#define TICKS_BEFORE_TIMEOUT 1
#define I2C_ACK_EN 0
#define I2C_BUF_LEN 8

#define TASK_STACK_SIZE 512 //default task stack size to use = 512 bytes, idk if we need to change


//TASK PRIORITIES:
//Each task is assigned a priority from 0 to ( configMAX_PRIORITIES - 1 ), 
//where configMAX_PRIORITIES is defined within FreeRTOSConfig.h. 
//greater number = higher priority
//tskIDLE_PRIORITY (idle task) has 0 priority


//task for ultrasonic sensor
void ultrasonicTask(void* param) {
    //setup
    ultrasonic_sensor_t ut = {
        .trigger_pin = UT_TRIGGER_PIN,
        .echo_pin = UT_ECHO_PIN,
    };
    esp_err_t err = ultrasonic_init(&ut);

    if (err != ESP_OK) {
        printf("Error in initializing ultrasonic sensor.\n");
        vTaskDelete(NULL); //causes calling task to be deleted
    }

    uint32_t utDist = 0;
    esp_err_t utMeasureErr;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); //set to 100ms for now
    BaseType_t xWasDelayed;

    
    TickType_t xLastWakeTime = xTaskGetTickCount(); //after initialization will be auto updated in xTaskDelayUntil()
    for ( ;; ) {
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency); //will delay until xFrequency ticks from lastWakeUpTime

        if (xWasDelayed == pdFALSE) {
            //printf("Error: ultrasonic task not delayed\n");
        }

        utMeasureErr = ultrasonic_measure_cm(&ut, 150, &utDist); //device pointer, max dist to measure in cm, resulting distance
        // if (utMeasureErr == ESP_ERR_ULTRASONIC_PING) {
        //     printf("invalid state error\n");
        // } else if (utMeasureErr == ESP_ERR_ULTRASONIC_PING_TIMEOUT) {
        //     printf("ping timeout error\n");
        // } else if (utMeasureErr == ESP_ERR_ULTRASONIC_ECHO_TIMEOUT) {
        //     printf("echo timeout error\n");
        // } else {
        //     printf("measured distance: %lu cm\n", utDist);
        // }

        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}


static void IRAM_ATTR handleJetsonInput(void* arg) {

    
}


void app_main() {
    //spawn tasks in app_main


    //ULTRASONIC SENSOR GPIO + TASK SETUP
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << UT_TRIGGER_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << UT_ECHO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    TaskHandle_t utTaskHandle = NULL;
    static uint8_t ucParameterToPass; //parameter must exist for lifetime of task (so it is static)
    //xTaskCreate(ultrasonicTask, "ultrasonic", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY+1, &utTaskHandle); //lowest priority+1

    
    //IMU GPIO + TASK SETUP



    //SERIAL I2C CONNECTION BETWEEN ESP AND JETSON
    // i2c_config_t i2c_conf = {
    //     .mode = I2C_MODE_MASTER, //esp = master
    //     .sda_io_num = ESP32_I2C_PIN_SDA,
    //     .scl_io_num = ESP32_I2C_PIN_SCL,
    //     .sda_pullup_en = GPIO_PULLUP_DISABLE, //i think jetson has pullups already on i2c
    //     .scl_pullup_en = GPIO_PULLUP_DISABLE,
    //     .master.clk_speed = I2C_MASTER_FREQ_HZ,
    // };
    // esp_err_t err = i2c_param_config(I2C_NUM_0, &i2c_conf); //port 0
    // err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0); //port 0

    // char buf[I2C_BUF_LEN];
    // memset(buf, 'b', I2C_BUF_LEN);

    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // err = i2c_master_start(cmd);
    // err = i2c_master_write_byte(cmd, JETSON_SLAVE_ADDRESS, I2C_ACK_EN);
    // err = i2c_master_write(cmd, (uint8_t*) buf, I2C_BUF_LEN, I2C_ACK_EN);
    // err = i2c_master_stop(cmd);
    // err = i2c_master_cmd_begin(I2C_NUM_0, cmd, TICKS_BEFORE_TIMEOUT);
    // i2c_cmd_link_delete(cmd);

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_SLAVE, //esp = slave
        .sda_io_num = ESP32_I2C_PIN_SDA,
        .scl_io_num = ESP32_I2C_PIN_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE, //i think jetson has pullups already on i2c
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = ESP_SLAVE_ADDRESS, //DO NOT SET SLAVE MAX SPEED IT MESSES STUFF UP
    };
    esp_err_t err = i2c_param_config(I2C_NUM_0, &i2c_conf); //port 0
    err = i2c_driver_install(I2C_NUM_0, I2C_MODE_SLAVE, 2048, 2048, 0); //port 0

    char buf[2048];
    memset(buf, 0, 2048);



    gpio_install_isr_service(0);
    gpio_isr_handler_add(JETSON_INPUT, handleJetsonInput, (void*) JETSON_INPUT);

    // io_conf.pin_bit_mask = (1ULL << 13);
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // gpio_config(&io_conf);

    // int currVal = 0;
    while (1) {
        // currVal = !currVal;
        // gpio_set_level(13, currVal);
        // vTaskDelay(5000 / portTICK_PERIOD_MS);
        int res = i2c_slave_read_buffer(I2C_NUM_0, (uint8_t*) buf, 2048, 1000 / portTICK_PERIOD_MS);
        printf("Received data: ");
        for (int i = 0; i < 2048; i++) {
            //printf("0x%02X", buf[i]);
            printf("%d", buf[i]);
        }
        printf("\n");
        printf("bytes read: %d\n", res);
        memset(buf, 0, 2048); //reset buffer
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

}