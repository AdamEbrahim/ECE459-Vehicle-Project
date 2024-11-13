#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "esp_err.h"
#include <string.h>

//HC-SR04 ultrasonic sensor library
#include <ultrasonic.h>

#include <driver/i2c.h>

//IMU stuff
#include "mpu6050.h"

//pwm drivers
#include <driver/ledc.h>
#include <math.h>

//ultrasonic stuff
#define LED_PIN 13
#define UT_TRIGGER_PIN 26
#define UT_ECHO_PIN 25
#define JETSON_INPUT 33

//I2C stuff
#define ESP32_I2C_SLAVE_PIN_SDA 23
#define ESP32_I2C_SLAVE_PIN_SCL 22
#define ESP_SLAVE_ADDRESS 0x48
#define I2C_SLAVE_PORT I2C_NUM_0
#define TICKS_BEFORE_TIMEOUT 1
#define I2C_ACK_EN 0
#define I2C_BUF_LEN 2048

#define ESP32_I2C_MASTER_PIN_SDA 21
#define ESP32_I2C_MASTER_PIN_SCL 14
#define I2C_MASTER_PORT I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 400000

//PWM stuff
#define MOTOR_PWM_CHANNEL_LEFT LEDC_CHANNEL_1 //attach left motor to ledc channel 1
#define MOTOR_PWM_CHANNEL_RIGHT LEDC_CHANNEL_2
#define MOTOR_PWM_TIMER LEDC_TIMER_1
#define MOTOR_PWM_BIT_NUM LEDC_TIMER_10_BIT
#define RIGHT_PWM_PIN 19
#define LEFT_PWM_PIN 21


#define TASK_STACK_SIZE 2048 //2048 seems to be working, increase if getting stack overflow error


//buffer for reading I2C data
char buf[I2C_BUF_LEN];

//task handles
TaskHandle_t motorTaskHandle; //motor
TaskHandle_t utTaskHandle; //ultrasonic
TaskHandle_t imuTaskHandle; //imu

//other global 


//TASK PRIORITIES:
//Each task is assigned a priority from 0 to ( configMAX_PRIORITIES - 1 ), 
//where configMAX_PRIORITIES is defined within FreeRTOSConfig.h. 
//greater number = higher priority
//tskIDLE_PRIORITY (idle task) has 0 priority


//setup function for motor pwm signal
void motor_pwm_init(void)
{
    ledc_channel_config_t ledc_channel_left = {0}, ledc_channel_right = {0};
    ledc_channel_left.gpio_num = LEFT_PWM_PIN;
    ledc_channel_left.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel_left.channel = MOTOR_PWM_CHANNEL_LEFT;
    ledc_channel_left.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_left.timer_sel = MOTOR_PWM_TIMER;
    ledc_channel_left.duty = 0;
	
    ledc_channel_right.gpio_num = RIGHT_PWM_PIN;
    ledc_channel_right.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel_right.channel = MOTOR_PWM_CHANNEL_RIGHT;
    ledc_channel_right.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_right.timer_sel = MOTOR_PWM_TIMER;
    ledc_channel_right.duty = 0;
	
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.duty_resolution = MOTOR_PWM_BIT_NUM;
    ledc_timer.timer_num = MOTOR_PWM_TIMER;
    ledc_timer.freq_hz = 25000; //25kHz right now, can change
	
    ledc_channel_config(&ledc_channel_left);
    ledc_channel_config(&ledc_channel_right);
    ledc_timer_config(&ledc_timer);
}

//function to set motor pwm signal
void motor_pwm_set(float left_duty_fraction, float right_duty_fraction) {
	uint32_t max_duty = (1 << MOTOR_PWM_BIT_NUM) - 1;
	uint32_t left_duty = lroundf(left_duty_fraction * (float)max_duty);
	uint32_t right_duty = lroundf(right_duty_fraction * (float)max_duty);
	
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_PWM_CHANNEL_LEFT, left_duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_PWM_CHANNEL_LEFT);

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_PWM_CHANNEL_RIGHT, right_duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_PWM_CHANNEL_RIGHT);
}



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
        
        //if detect object less than 10cm away, stop motors
        if (utDist < 10) {
            vTaskSuspend(motorTaskHandle);
            motor_pwm_set(0, 0);
        } else { //if no object detected in front of us within 10 cm, resume motors
            vTaskResume(motorTaskHandle);
        }
        
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


void imuTask(void* param) {
    //setup
    mpu6050_handle_t mpu6050_dev = mpu6050_create(I2C_MASTER_PORT, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050_dev, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050_dev);

    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    complimentary_angle_t complimentary_angle;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); //set to 100ms for now
    BaseType_t xWasDelayed;


    TickType_t xLastWakeTime = xTaskGetTickCount();
    for ( ;; ) {
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xWasDelayed == pdFALSE) {
            //printf("Error: imu task not delayed\n");
        }

        mpu6050_get_acce(mpu6050_dev, &acce);
        mpu6050_get_gyro(mpu6050_dev, &gyro);
        mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle);

    }

}


void motorTask(void* param) {
    //PWM setup
    motor_pwm_init();
    motor_pwm_set(0, 0);

    int res;
    const TickType_t xFrequency = pdMS_TO_TICKS(50); //set to 50ms for now
    BaseType_t xWasDelayed;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for ( ;; ) {
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xWasDelayed == pdFALSE) {
            //printf("Error: motor task not delayed\n");
        }

        //I2C read into buffer
        res = i2c_slave_read_buffer(I2C_SLAVE_PORT, (uint8_t*) buf, I2C_BUF_LEN, 0);
        // printf("Received data: ");
        // for (int i = 0; i < 2048; i++) {
        //     //printf("0x%02X", buf[i]);
        //     printf("%d", buf[i]);
        // }
        // printf("\n");
        // printf("bytes read: %d\n", res);

        if (buf[0] == 1) {
            motor_pwm_set(0.9, 0.9);
        } else if (buf[0] == 2) {
            motor_pwm_set(0, 0.9);
        } else if (buf[0] == 3) {
            motor_pwm_set(0.9, 0);
        }

        memset(buf, 0, I2C_BUF_LEN); //reset buffer
    }
}



static void IRAM_ATTR handleJetsonInput(void* arg) {

    
}


void app_main() {
    //spawn tasks in app_main

    //----TASK SETUP STUFF----//

    //MOTOR TASK SETUP (must setup before ultrasonic)
    motorTaskHandle = NULL;
    static uint8_t ucParameterToPass2; //parameter must exist for lifetime of task (so it is static)
    xTaskCreate(motorTask, "motor", TASK_STACK_SIZE, &ucParameterToPass2, tskIDLE_PRIORITY+2, &motorTaskHandle); //lowest priority+2


    //ULTRASONIC TASK SETUP
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << UT_TRIGGER_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << UT_ECHO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    utTaskHandle = NULL;
    static uint8_t ucParameterToPass;
    xTaskCreate(ultrasonicTask, "ultrasonic", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY+1, &utTaskHandle); //lowest priority+1


    //IMU TASK SETUP
    imuTaskHandle = NULL;
    static uint8_t ucParameterToPass3; 
    xTaskCreate(imuTask, "imu", TASK_STACK_SIZE, &ucParameterToPass3, tskIDLE_PRIORITY+1, &imuTaskHandle); //lowest priority+1


    //----I2C SETUP STUFF----//

    //port 0, I2C slave
    i2c_config_t i2c_conf = { 
        .mode = I2C_MODE_SLAVE, //esp = slave
        .sda_io_num = ESP32_I2C_SLAVE_PIN_SDA,
        .scl_io_num = ESP32_I2C_SLAVE_PIN_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE, //i think jetson has pullups already on i2c
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = ESP_SLAVE_ADDRESS, //DO NOT SET SLAVE MAX SPEED IT MESSES STUFF UP
    };
    esp_err_t err = i2c_param_config(I2C_SLAVE_PORT, &i2c_conf); //port 0
    err = i2c_driver_install(I2C_SLAVE_PORT, I2C_MODE_SLAVE, I2C_BUF_LEN, I2C_BUF_LEN, 0); //port 0

    memset(buf, 0, I2C_BUF_LEN); //set global buffer for received I2C data to 0 to start

    //port 1, I2C master
    i2c_config_t i2c_conf1 = { 
        .mode = I2C_MODE_MASTER, //esp = master
        .sda_io_num = ESP32_I2C_MASTER_PIN_SDA,
        .scl_io_num = ESP32_I2C_MASTER_PIN_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE, //connect our own pullups
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ, //idk if we need this
    };
    err = i2c_param_config(I2C_MASTER_PORT, &i2c_conf1); //port 1
    err = i2c_driver_install(I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0); //port 1


    //----OTHER INTERRUPT AND MISC STUFF----//

    gpio_install_isr_service(0);
    gpio_isr_handler_add(JETSON_INPUT, handleJetsonInput, (void*) JETSON_INPUT);

    // io_conf.pin_bit_mask = (1ULL << 13);
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // gpio_config(&io_conf);
    // int currVal = 0;
    // gpio_set_level(13, currVal);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);


    //----WHILE LOOP----//
    // while (1) {
        
    // }

}


// I2C MASTER CODE FOR ESP32
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