#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "esp_err.h"

//HC-SR04 ultrasonic sensor library
#include <ultrasonic.h>

#include <driver/i2c.h>

#define LED_PIN 13
#define UT_TRIGGER_PIN 26
#define UT_ECHO_PIN 25
#define JETSON_INPUT 33

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
    xTaskCreate(ultrasonicTask, "ultrasonic", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY+1, &utTaskHandle); //lowest priority+1

    
    //IMU GPIO + TASK SETUP



    //SERIAL I2C CONNECTION BETWEEN ESP AND JETSON
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
    };


    gpio_install_isr_service(0);
    gpio_isr_handler_add(JETSON_INPUT, handleJetsonInput, (void*) JETSON_INPUT);


    // while (1) {
        
    // }

}