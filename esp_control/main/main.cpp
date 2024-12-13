extern "C" {
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "esp_err.h"
#include "esp_log.h"
#include <string.h>
#include <esp_timer.h>

//HC-SR04 ultrasonic sensor library
#include <ultrasonic.h>

#include <driver/i2c.h>

//IMU stuff
//#include "mpu6050.h"

//pwm drivers
#include <driver/ledc.h>
//#include <math.h>

// #include "ahrs.h"
// #include "mpu9250.h"
// #include "calibrate.h"
// #include "common.h"
}

#include "Adafruit_MotorShield.h"
#include "SimplePID.h"
#include <cmath>
#include "angle.h"

//ultrasonic stuff
#define LED_PIN 13
#define UT_TRIGGER_PIN GPIO_NUM_26
#define UT_ECHO_PIN GPIO_NUM_25

//I2C stuff
#define ESP32_I2C_SLAVE_PIN_SDA 15
#define ESP32_I2C_SLAVE_PIN_SCL 14
#define ESP_SLAVE_ADDRESS 0x48
#define I2C_SLAVE_PORT I2C_NUM_0
#define TICKS_BEFORE_TIMEOUT 1
#define I2C_ACK_EN 0
#define I2C_BUF_LEN 2048

#define ESP32_I2C_MASTER_PIN_SDA 23
#define ESP32_I2C_MASTER_PIN_SCL 22
#define I2C_MASTER_PORT I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 400000

#define MOTOR_DRIVER_I2C_ADDRESS 0x60

//PWM stuff
#define MOTOR_PWM_CHANNEL_LEFT LEDC_CHANNEL_1
#define MOTOR_PWM_CHANNEL_RIGHT LEDC_CHANNEL_2
#define MOTOR_PWM_TIMER LEDC_TIMER_1
#define MOTOR_PWM_BIT_NUM LEDC_TIMER_10_BIT
#define RIGHT_PWM_PIN 21
#define LEFT_PWM_PIN 19

//motor + encoder stuff
#define RIGHT_MOTOR_ENCODER_A GPIO_NUM_21
#define RIGHT_MOTOR_ENCODER_B GPIO_NUM_19
#define LEFT_MOTOR_ENCODER_A GPIO_NUM_33
#define LEFT_MOTOR_ENCODER_B GPIO_NUM_32
const gpio_num_t encoderAPins[] = {RIGHT_MOTOR_ENCODER_A, LEFT_MOTOR_ENCODER_A}; //right motor, left motor
const gpio_num_t encoderBPins[] = {RIGHT_MOTOR_ENCODER_B, LEFT_MOTOR_ENCODER_B}; //right motor, left motor

volatile int posiMotors[] = {0, 0}; //current position from encoder; right motor, left motor
float targetPosMotors_f[] = {0,0}; //target position of motors as float; right motor, left motor
long targetPosMotors[] = {0,0}; //target position of motors; right motor, left motor

Adafruit_DCMotor* motors[2]; //actual motor pointers; right motor, left motor
SimplePID pid[2]; //pid control; right motor, left motor

uint8_t currUserInputSpeed = 0; //0-255
bool canGoForward = true; //global var indicating if can move forward based on if ultrasonic detecting something in front

//66mm diameter wheels
#define ENCODER_PULSES_PER_ROTATION 43.8*16
#define WHEEL_CM_PER_ROTATION 20.7345


#define TASK_STACK_SIZE 2048 //2048 seems to be working, increase if getting stack overflow error

typedef struct motor_control_packet { //read inside the I2C buffer, packet sent from Jetson for direction control
    char direction; //set to 0 for red traffic light, no change for green traffic light,
    char speed;
    char detection; // stop sign detection = 1
} motor_control_packet_t;

//buffer for reading I2C data
char buf[I2C_BUF_LEN];

//task handles
TaskHandle_t motorTaskHandle; //motor
TaskHandle_t utTaskHandle; //ultrasonic
TaskHandle_t imuTaskHandle; //imu

//other global 
static const char *TAG = "MAIN";
float global_yaw;
bool obstacle_avoidance_override = false;
static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;
float targetAngleDeg = 90;
float targetAngleRad = targetAngleDeg * M_PI / 180.0; 
int imu_adjust_dir;
int imu_adjust_pwr;
void mpu6050(void *pvParameters); //no header file so declare function from mpu6050.cpp


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
        //printf("Error in initializing ultrasonic sensor.\n");
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

        utMeasureErr = ultrasonic_measure_cm(&ut, 75, &utDist); //device pointer, max dist to measure in cm, resulting distance
        
        //if no object detected in front of us within 15 cm, resume motors (happens both when detected distance > 15 or when echo times out)
        if (utMeasureErr == ESP_ERR_ULTRASONIC_ECHO_TIMEOUT || utDist > 15) {
            //vTaskResume(motorTaskHandle);
            canGoForward = true; //make it so car can go forward 
            //gpio_set_level(GPIO_NUM_13, 0);
        } else if (utDist < 15) { //if detect object less than 15cm away, dont allow motors to go forward
            //vTaskSuspend(motorTaskHandle); //momentarily suspend motor task so no concurrency issues where motor task preempts
            canGoForward = false; //make it so user can't go forward (1 writer, multiple readers so no concurrency issues i think)
            obstacle_avoidance_override = true;
            //gpio_set_level(GPIO_NUM_13, 1);
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


// void imuTask(void* param) {
//     //setup
//     mpu6050_handle_t mpu6050_dev = mpu6050_create(I2C_MASTER_PORT, MPU6050_I2C_ADDRESS);
//     mpu6050_config(mpu6050_dev, ACCE_FS_4G, GYRO_FS_500DPS);
//     mpu6050_wake_up(mpu6050_dev);

//     mpu6050_acce_value_t acce;
//     mpu6050_gyro_value_t gyro;
//     complimentary_angle_t complimentary_angle;
//     const TickType_t xFrequency = pdMS_TO_TICKS(100); //set to 100ms for now
//     BaseType_t xWasDelayed;


//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     for ( ;; ) {
//         xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

//         if (xWasDelayed == pdFALSE) {
//             //printf("Error: imu task not delayed\n");
//         }

//         mpu6050_get_acce(mpu6050_dev, &acce);
//         mpu6050_get_gyro(mpu6050_dev, &gyro);
//         mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle);

//     }

// }


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

//deltaT in seconds, velocity in cm/s
void setTarget(float t, float deltaT, float velocity) {
    float posChange[2] = {0.0, 0.0}; //how much the motor pos should have changed to get some speed v; right motor, left motor

    for (int i = 0; i < 2; i++) {
        posChange[i] = velocity * deltaT * (1 / WHEEL_CM_PER_ROTATION) * ENCODER_PULSES_PER_ROTATION;
    }

    //update what the targetPos of motors is to maintain speed v
    for (int i = 0; i < 2; i++) {
        targetPosMotors_f[i] = targetPosMotors_f[i] + posChange[i];
    }
    //NOT SURE IF BELOW IS RIGHT, MIGHT HAVE TO MAKE 1 NEGATIVE
    targetPosMotors[0] = (long) targetPosMotors_f[0]; //right motor
    targetPosMotors[1] = -1 * ((long) targetPosMotors_f[1]); //left motor
    //targetPosMotors[1] = 10;
}

//dir = direction, pwr = value from 0-255 to set motor speed to
void setMotor(Adafruit_DCMotor* motorHandle, int dir, int pwr) {
    if (dir == 1) {
        motorHandle->run(FORWARD);
    } else if (dir == -1) {
        motorHandle->run(BACKWARD);
    } else {
        motorHandle->run(RELEASE);
    }

    motorHandle->setSpeed(pwr);
}

void motorTask2(void* param) {

    long prevT = esp_timer_get_time(); //microseconds; used for time steps in pid control (derivative, etc)
    long pos[2]; //every iteration read the motor pos into here so interrupts that update pos don't cause too much discrepancy

    const TickType_t xFrequency = pdMS_TO_TICKS(50); //set to 100ms for now
    BaseType_t xWasDelayed;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for ( ;; ) {
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xWasDelayed == pdFALSE) {
            //printf("Error: motor task not delayed\n");
        }

        long currT = esp_timer_get_time(); //get time in microseconds
        float deltaT = ((float) (currT - prevT))/( 1.0e6 ); //get difference in time between currT and prevT in seconds
        prevT = currT; //adjust prevT for next iteration

        //figure out what target position should be if some deltaT has passed based on some speed
        setTarget(currT/1.0e6, deltaT, 6); //currTime in seconds, deltaT in seconds, velocity (cm/s)

        //get curr motor positions (IDK IF I NEED TO DO THIS ATOMICALLY OR WITH LOCKS)
        for (int i = 0; i < 2; i++) {
            pos[i] = posiMotors[i];
        }
 
        ESP_LOGI(TAG, "RIGHT MOTOR POS: %ld", pos[0]);
        ESP_LOGI(TAG, "LEFT MOTOR POS: %ld", pos[1]);

        //use pid controllers to figure out what control signals need to be sent to each motor based on curr motor positions and target positions
        for (int i = 0; i < 2; i++) {
            int dir, pwr; //dir = direction, pwr = value from 0-255 to set motor speed to
            pid[i].evalu(pos[i], targetPosMotors[i], deltaT, pwr, dir, i); //figure out control signals
            setMotor(motors[i], dir, pwr); //actually set motor speed and direction
        }



    }
}

void imu_motor_adjust(void* param) {
    SimplePID anglePID;
    anglePID.setParams(3,0,0,255);
    long prevT = esp_timer_get_time(); //microseconds; used for time steps in pid control (derivative, etc)
    // float targetAngleDeg = 90;
    // float targetAngleRad = targetAngleDeg * M_PI / 180.0; 

    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    BaseType_t xWasDelayed;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for ( ;; ) {
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xWasDelayed == pdFALSE) {
            //printf("Error: motor task not delayed\n");
            ESP_LOGI(TAG, "Error: imu motor adjust task not delayed");
        }

        long currT = esp_timer_get_time(); //get time in microseconds
        float deltaT = ((float) (currT - prevT))/( 1.0e6 ); //get difference in time between currT and prevT in seconds
        prevT = currT; //adjust prevT for next iteration

        //use pid controllers to figure out what control signals need to be sent to each motor based on curr motor positions and target positions
        int dir, pwr; //dir = direction, pwr = value from 0-255 to set motor speed to
        // anglePID.evaluAngle(global_yaw * (M_PI / 180), targetAngleRad, deltaT, pwr, dir); //figure out control signals
        // ESP_LOGI(TAG, "input yaw: %f, target: %f, deltaT: %f, pwr: %d, dir: %d", global_yaw * (M_PI / 180), targetAngleRad, deltaT, pwr, dir);
        anglePID.evaluAngle(global_yaw, targetAngleDeg, deltaT, pwr, dir); //figure out control signals
        ESP_LOGI(TAG, "input yaw: %f, target: %f, deltaT: %f, pwr: %d, dir: %d", global_yaw, targetAngleDeg, deltaT, pwr, dir);
        imu_adjust_dir = dir;
        imu_adjust_pwr = pwr;


    }


}

//MOTORTASK2_NOENCODER VERSION WITH IMU ADJUSTMENTS TO GO IN STRAIGHT LINE (IMU YAW ANGLE READINGS ARE NOT ACCURATE SO DOES NOT WORK)
void motorTask2_noEncoder(void* param) {
    int res;
    int numCyclesNoDirection = 0; //count how many consecutive cycles no data is read in buffer to verify data has actually stopped being sent
    int noDirectionCyclesBeforeStop = 3; 
    const TickType_t xFrequency = pdMS_TO_TICKS(200); //set to 100ms for now to allow enough time for jetson to send something over i2c into buffer
    BaseType_t xWasDelayed;

    const TickType_t stopSignDetectionFrequency = pdMS_TO_TICKS(12000); //12 seconds between new stop sign detections
    const TickType_t stopSignStopTime = pdMS_TO_TICKS(4000); //4 second stop at stop sign
    TickType_t stopSignFirstDetectedTime = 0;

    bool prevWasForward = false;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for ( ;; ) {
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xWasDelayed == pdFALSE) {
            //printf("Error: motor task not delayed\n");
        }

        // ESP_LOGI(TAG, "RIGHT MOTOR POS: %d", posiMotors[0]);
        // ESP_LOGI(TAG, "LEFT MOTOR POS: %d", posiMotors[1]);

        // if (obstacle_avoidance_override) { //no user control if in obstacle avoidance mode

        //     if (return_to_user_control) { //TODO
        //         memset(buf, 0, I2C_BUF_LEN); //reset user control buffer
        //     }

        //     continue;
        // }

        //I2C read into buffer
        res = i2c_slave_read_buffer(I2C_SLAVE_PORT, (uint8_t*) buf, I2C_BUF_LEN, 0); //res = # bytes read
        // printf("Received data: ");
        for (int i = 0; i < 6; i++) {
            //printf("0x%02X", buf[i]);
            ESP_LOGI(TAG, "Buffer %d: %d", i, buf[i]);

        }

        // printf("\n");
        // printf("bytes read: %d\n", res);

        motor_control_packet_t* control = (motor_control_packet_t*) buf;
        ESP_LOGI(TAG, "received control: direction=%d   speed=%d    detection=%d", control->direction, control->speed, control->detection);

        if (control->detection == 1 && xTaskGetTickCount() - stopSignFirstDetectedTime >= stopSignDetectionFrequency) { //new detection of stop sign
            stopSignFirstDetectedTime = xTaskGetTickCount();
            ESP_LOGI(TAG, "NEW STOP SIGN DETECTION DETECTIONDETECTIONDETECTIONDETECTIONDETECTIONDETECTION");
        }


        if (xTaskGetTickCount() - stopSignFirstDetectedTime < stopSignStopTime) { //if still a valid stop sign detection (first 4 seconds) should overwrite all control
            motors[0]->run(RELEASE);
            motors[1]->run(RELEASE);
            ESP_LOGI(TAG, "VALID STOP SIGN  ");
        
        } else if (control->direction == 1) { //forward
            if (!prevWasForward) {
                targetAngleDeg = global_yaw;
                targetAngleRad = targetAngleDeg * M_PI / 180.0; 
            }

            prevWasForward = true;

            if (canGoForward) {
                if (imu_adjust_dir == 1) {
                    int rPwr = -1 * imu_adjust_pwr + control->speed;
                    int rDir = 1;
                    if (rPwr < 0) {
                        rDir = -1;
                        rPwr *= -1;
                    }
                    int lPwr = imu_adjust_pwr + control->speed;
                    if (lPwr > 255) {
                        lPwr = 255;
                    }
                    setMotor(motors[0], rDir, rPwr); //actually set motor speed and direction
                    setMotor(motors[1], 1, lPwr); //actually set motor speed and direction
                } else {
                    int lPwr = -1 * imu_adjust_pwr + control->speed;
                    int lDir = 1;
                    if (lPwr < 0) {
                        lDir = -1;
                        lPwr *= -1;
                    }
                    int rPwr = imu_adjust_pwr + control->speed;
                    if (rPwr > 255) {
                        rPwr = 255;
                    }
                    setMotor(motors[0], 1, rPwr); //actually set motor speed and direction
                    setMotor(motors[1], lDir, lPwr); //actually set motor speed and direction

                }
                numCyclesNoDirection = 0;
                memset(buf, 0, I2C_BUF_LEN); //reset buffer
                continue;

                
                // motors[0]->run(FORWARD);
                // motors[1]->run(FORWARD);
            } else { //trying to move forward but cant; stop motors, reset speed
                motors[0]->run(RELEASE);
                motors[1]->run(RELEASE);
                // currUserInputSpeed = 0;
                // motors[0]->setSpeed(currUserInputSpeed);
                // motors[1]->setSpeed(currUserInputSpeed);
            }
            numCyclesNoDirection = 0;
        } else if (control->direction == 2) { //left
            prevWasForward = false;
            motors[0]->run(FORWARD);
            motors[1]->run(BACKWARD);
            numCyclesNoDirection = 0;
        } else if (control->direction == 3) { //right
            prevWasForward = false;
            motors[0]->run(BACKWARD);
            motors[1]->run(FORWARD);
            numCyclesNoDirection = 0;
        } else if (control->direction == 4) { //backwards
            prevWasForward = false;
            motors[0]->run(BACKWARD);
            motors[1]->run(BACKWARD);
            numCyclesNoDirection = 0;
        } else {
            //prevWasForward = false;
            // numCyclesNoDirection++;
            // ESP_LOGI(TAG, "CYCLE NO DETECTION: NUM CYCLES NO=%d", numCyclesNoDirection);
            // if (numCyclesNoDirection >= noDirectionCyclesBeforeStop) { //only stop if detected nothing in buffer for some number of cycles
                motors[0]->run(RELEASE);
                motors[1]->run(RELEASE);
            // }
        }

        //set speed
        motors[0]->setSpeed(control->speed);
        motors[1]->setSpeed(control->speed);
        
        
        // else if (control->direction == 5) { //faster
        //     if (currUserInputSpeed < 255) { //only if speed less than 255
        //         currUserInputSpeed+=5;
        //         motors[0]->setSpeed(currUserInputSpeed);
        //         motors[1]->setSpeed(currUserInputSpeed);
        //     }
        // } else if (control->direction == 6) { //slower
        //     if (currUserInputSpeed > 0) { //only if speed greater than 0
        //         currUserInputSpeed-=5;
        //         motors[0]->setSpeed(currUserInputSpeed);
        //         motors[1]->setSpeed(currUserInputSpeed);
        //     }
        // }

        memset(buf, 0, I2C_BUF_LEN); //reset buffer
    }
}

// void motorTask2_noEncoder(void* param) {
//     int res;
//     int numCyclesNoDirection = 0; //count how many consecutive cycles no data is read in buffer to verify data has actually stopped being sent
//     int noDirectionCyclesBeforeStop = 3; 
//     const TickType_t xFrequency = pdMS_TO_TICKS(200); //set to 100ms for now to allow enough time for jetson to send something over i2c into buffer
//     BaseType_t xWasDelayed;

//     const TickType_t stopSignDetectionFrequency = pdMS_TO_TICKS(12000); //12 seconds between new stop sign detections
//     const TickType_t stopSignStopTime = pdMS_TO_TICKS(4000); //4 second stop at stop sign
//     TickType_t stopSignFirstDetectedTime = 0;


//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     for ( ;; ) {
//         xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

//         if (xWasDelayed == pdFALSE) {
//             //printf("Error: motor task not delayed\n");
//         }

//         // ESP_LOGI(TAG, "RIGHT MOTOR POS: %d", posiMotors[0]);
//         // ESP_LOGI(TAG, "LEFT MOTOR POS: %d", posiMotors[1]);

//         // if (obstacle_avoidance_override) { //no user control if in obstacle avoidance mode

//         //     if (return_to_user_control) { //TODO
//         //         memset(buf, 0, I2C_BUF_LEN); //reset user control buffer
//         //     }

//         //     continue;
//         // }

//         //I2C read into buffer
//         res = i2c_slave_read_buffer(I2C_SLAVE_PORT, (uint8_t*) buf, I2C_BUF_LEN, 0); //res = # bytes read
//         // printf("Received data: ");
//         for (int i = 0; i < 6; i++) {
//             //printf("0x%02X", buf[i]);
//             ESP_LOGI(TAG, "Buffer %d: %d", i, buf[i]);

//         }

//         // printf("\n");
//         // printf("bytes read: %d\n", res);

//         motor_control_packet_t* control = (motor_control_packet_t*) buf;
//         ESP_LOGI(TAG, "received control: direction=%d   speed=%d    detection=%d", control->direction, control->speed, control->detection);

//         if (control->detection == 1 && xTaskGetTickCount() - stopSignFirstDetectedTime >= stopSignDetectionFrequency) { //new detection of stop sign
//             stopSignFirstDetectedTime = xTaskGetTickCount();
//             ESP_LOGI(TAG, "NEW STOP SIGN DETECTION DETECTIONDETECTIONDETECTIONDETECTIONDETECTIONDETECTION");
//         }


//         if (xTaskGetTickCount() - stopSignFirstDetectedTime < stopSignStopTime) { //if still a valid stop sign detection (first 4 seconds) should overwrite all control
//             motors[0]->run(RELEASE);
//             motors[1]->run(RELEASE);
//             ESP_LOGI(TAG, "VALID STOP SIGN  ");
        
//         } else if (control->direction == 1) { //forward


//             if (canGoForward) {    
//                 motors[0]->run(FORWARD);
//                 motors[1]->run(FORWARD);
//                 if (control->speed == 36) { //34, 86.5, 140
//                     motors[0]->setSpeed(34);
//                     motors[1]->setSpeed(36);

//                 } else if (control->speed == 90) {
//                     motors[0]->setSpeed(86.5);
//                     motors[1]->setSpeed(90);

//                 } else if (control->speed == 144) {
//                     motors[0]->setSpeed(138);
//                     motors[1]->setSpeed(144);

//                 }

//                 numCyclesNoDirection = 0;
                
//                 memset(buf, 0, I2C_BUF_LEN); //reset buffer
//                 continue;
//             } else { //trying to move forward but cant; stop motors, reset speed
//                 motors[0]->run(RELEASE);
//                 motors[1]->run(RELEASE);
//                 // currUserInputSpeed = 0;
//                 // motors[0]->setSpeed(currUserInputSpeed);
//                 // motors[1]->setSpeed(currUserInputSpeed);
//             }
//             numCyclesNoDirection = 0;
//         } else if (control->direction == 2) { //left
//             //prevWasForward = false;
//             motors[0]->run(FORWARD);
//             motors[1]->run(RELEASE);
//             numCyclesNoDirection = 0;
//         } else if (control->direction == 3) { //right
//             //prevWasForward = false;
//             motors[0]->run(RELEASE);
//             motors[1]->run(FORWARD);
//             numCyclesNoDirection = 0;
//         } else if (control->direction == 4) { //backwards
//             //prevWasForward = false;
//             motors[0]->run(BACKWARD);
//             motors[1]->run(BACKWARD);
//             numCyclesNoDirection = 0;
//         } else {
//             //prevWasForward = false;
//             // numCyclesNoDirection++;
//             // ESP_LOGI(TAG, "CYCLE NO DETECTION: NUM CYCLES NO=%d", numCyclesNoDirection);
//             // if (numCyclesNoDirection >= noDirectionCyclesBeforeStop) { //only stop if detected nothing in buffer for some number of cycles
//                 motors[0]->run(RELEASE);
//                 motors[1]->run(RELEASE);
//             // }
//         }

//         //set speed
//         motors[0]->setSpeed(control->speed);
//         motors[1]->setSpeed(control->speed);
        
        
//         // else if (control->direction == 5) { //faster
//         //     if (currUserInputSpeed < 255) { //only if speed less than 255
//         //         currUserInputSpeed+=5;
//         //         motors[0]->setSpeed(currUserInputSpeed);
//         //         motors[1]->setSpeed(currUserInputSpeed);
//         //     }
//         // } else if (control->direction == 6) { //slower
//         //     if (currUserInputSpeed > 0) { //only if speed greater than 0
//         //         currUserInputSpeed-=5;
//         //         motors[0]->setSpeed(currUserInputSpeed);
//         //         motors[1]->setSpeed(currUserInputSpeed);
//         //     }
//         // }

//         memset(buf, 0, I2C_BUF_LEN); //reset buffer
//     }
// }


static void IRAM_ATTR motorReadEncoderRight(void* arg) {
    int i = 0; //right motor index
    int val = gpio_get_level(encoderBPins[i]);

    if (val > 0) {
        posiMotors[i]++;
    } else{
        posiMotors[i]--;
    }
    
}

static void IRAM_ATTR motorReadEncoderLeft(void* arg) {
    int i = 1; //left motor index
    int val = gpio_get_level(encoderBPins[i]);

    if (val > 0) {
        posiMotors[i]++;
        //gpio_set_level(GPIO_NUM_13, 1);
    } else{
        posiMotors[i]--;
    }
    
}

// // calibration_t cal = {
// //     .mag_offset = {.x = 47.369141, .y = -13.148438, .z = -16.765625},
// //     .mag_scale = {.x = 0.858812, .y = 1.482491, .z = 0.861282},
// //     .gyro_bias_offset = {.x = 0.101624, .y = 0.698818, .z = -0.957152},
// //     .accel_offset = {.x = 0.216052, .y = 0.163032, .z = -0.601574},
// //     .accel_scale_lo = {.x = 1.078030, .y = 1.072876, .z = 0.673028},
// //     .accel_scale_hi = {.x = -0.903445, .y = -0.923606, .z = -1.351972}};
// // calibration_t cal = {
// //     .mag_offset = {.x = 42.572266, .y = -28.687500, .z = -17.921875},
// //     .mag_scale = {.x = 0.978342, .y = 0.990303, .z = 1.032982},
// //     .gyro_bias_offset = {.x = 0.142495, .y = 0.715907, .z = -1.036259},
// //     .accel_offset = {.x = 0.179734, .y = 0.134102, .z = -0.622451},
// //     .accel_scale_lo = {.x = 1.078384, .y = 1.078952, .z = 0.672689},
// //     .accel_scale_hi = {.x = -0.911362, .y = -0.922406, .z = -1.358516}};

//     calibration_t cal = {
//     .mag_offset = {.x = 35.376953, .y = -25.699219, .z = -15.609375},
//     .mag_scale = {.x = 0.975774, .y = 0.927887, .z = 1.114262},
//     .gyro_bias_offset = {.x = 0.125058, .y = 0.574033, .z = -0.989811},
//     .accel_offset = {.x = 0.241969, .y = 0.188752, .z = -0.611557},
//     .accel_scale_lo = {.x = 1.081904, .y = 1.079533, .z = 0.675943},
//     .accel_scale_hi = {.x = -0.914257, .y = -0.909528, .z = -1.329812}};

// /**
//  * Transformation:
//  *  - Rotate around Z axis 180 degrees
//  *  - Rotate around X axis -90 degrees
//  * @param  {object} s {x,y,z} sensor
//  * @return {object}   {x,y,z} transformed
//  */
// static void transform_accel_gyro(vector_t *v)
// {
//   float x = v->x;
//   float y = v->y;
//   float z = v->z;

//   v->x = -x;
//   v->y = -z;
//   v->z = -y;
// }

// /**
//  * Transformation: to get magnetometer aligned
//  * @param  {object} s {x,y,z} sensor
//  * @return {object}   {x,y,z} transformed
//  */
// static void transform_mag(vector_t *v)
// {
//   float x = v->x;
//   float y = v->y;
//   float z = v->z;

//   v->x = -y;
//   v->y = z;
//   v->z = -x;
// }

// void run_imu(void)
// {

//   i2c_mpu9250_init(&cal);
//   ahrs_init(SAMPLE_FREQ_Hz, 0.8);

//   uint64_t i = 0;
//     const TickType_t xFrequency = pdMS_TO_TICKS(50); //20Hz
//     BaseType_t xWasDelayed;

//     TickType_t xLastWakeTime = xTaskGetTickCount();
//   while (true)
//   {
//     xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);
//     if (xWasDelayed == pdFALSE) {
//         ESP_LOGI(TAG, "Error: IMU task not delayed");
//         //printf("Error: imu task not delayed\n");
//     }
//     vector_t va, vg, vm;

//     // Get the Accelerometer, Gyroscope and Magnetometer values.
//     ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

//     // Transform these values to the orientation of our device.
//     transform_accel_gyro(&va);
//     transform_accel_gyro(&vg);
//     transform_mag(&vm);

//     // Apply the AHRS algorithm
//     ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
//                 va.x, va.y, va.z,
//                 vm.x, vm.y, vm.z);

//     //CHANGE: CHECK IF I CAN DO THIS EVERY ITERATION
//     float heading, pitch, roll;
//     ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
//     global_yaw = heading;

//     // Print the data out every 10 items
//     if (i++ % 10 == 0)
//     {
//       float temp;
//       ESP_ERROR_CHECK(get_temperature_celsius(&temp));

//     //   float heading, pitch, roll;
//     //   ahrs_get_euler_in_degrees(&heading, &pitch, &roll);
//     //   global_yaw = heading;
//       ESP_LOGI(TAG, "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°, Temp %2.3f°C", heading, pitch, roll, temp);

//       // Make the WDT happy
//       vTaskDelay(0);
//     }

//     //pause();
//   }
// }

// static void imu_task(void *arg)
// {

// #ifdef CONFIG_CALIBRATION_MODE
//   calibrate_gyro();
//   calibrate_accel();
//   calibrate_mag();
// #else
//   run_imu();
// #endif

//   // Exit
//   vTaskDelay(100 / portTICK_PERIOD_MS);
//   i2c_driver_delete(I2C_MASTER_PORT);

//   vTaskDelete(NULL);
// }


extern "C" void app_main() {
    //spawn tasks in app_main
    // gpio_config_t io_conf2;
    // io_conf2.pin_bit_mask = (1ULL << GPIO_NUM_13);
    // io_conf2.mode = GPIO_MODE_OUTPUT;
    // gpio_config(&io_conf2);

    //----I2C SETUP STUFF----//

    //port 0, I2C slave
    i2c_config_t i2c_conf = { 
        .mode = I2C_MODE_SLAVE, //esp = slave
        .sda_io_num = ESP32_I2C_SLAVE_PIN_SDA,
        .scl_io_num = ESP32_I2C_SLAVE_PIN_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE, //i think jetson has pullups already on i2c
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .slave {
            .addr_10bit_en = 0,
            .slave_addr = ESP_SLAVE_ADDRESS, //DO NOT SET SLAVE MAX SPEED IT MESSES STUFF UP
        }
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
        .master {
            .clk_speed = I2C_MASTER_FREQ_HZ, //idk if we need this
        }
    };
    err = i2c_param_config(I2C_MASTER_PORT, &i2c_conf1); //port 1
    err = i2c_driver_install(I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0); //port 1

    //----TASK SETUP STUFF----//

    //MOTOR TASK SETUP (must setup before ultrasonic) AND ENCODER SETUP
    gpio_config_t encoder_io_conf = {
        .pin_bit_mask = (1ULL << RIGHT_MOTOR_ENCODER_A),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE, //positive edge interrupt
    };
    gpio_config(&encoder_io_conf);

    encoder_io_conf.pin_bit_mask = (1ULL << LEFT_MOTOR_ENCODER_A);
    encoder_io_conf.mode = GPIO_MODE_INPUT;
    encoder_io_conf.intr_type = GPIO_INTR_POSEDGE; //positive edge interrupt
    gpio_config(&encoder_io_conf);

    gpio_config_t encoder_io_conf2 = {
        .pin_bit_mask = (1ULL << RIGHT_MOTOR_ENCODER_B),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&encoder_io_conf2);

    encoder_io_conf2.pin_bit_mask = (1ULL << LEFT_MOTOR_ENCODER_B);
    encoder_io_conf2.mode = GPIO_MODE_INPUT;
    gpio_config(&encoder_io_conf2);


    gpio_install_isr_service(0); //allow interrupts
    gpio_isr_handler_add(encoderAPins[0], motorReadEncoderRight, (void*) encoderAPins[0]); //attach interrupt from right motor encoder A
    gpio_isr_handler_add(encoderAPins[1], motorReadEncoderLeft, (void*) encoderAPins[1]); //attach interrupt from left motor encoder A

    //initialize the motors
    Adafruit_MotorShield AFMS = Adafruit_MotorShield(MOTOR_DRIVER_I2C_ADDRESS, I2C_MASTER_PORT);
    Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); //port M1 on motor driver board
    Adafruit_DCMotor *rightMotor = AFMS.getMotor(3); //port M3 on motor driver board
    motors[0] = rightMotor;
    motors[1] = leftMotor;
    AFMS.begin();
    // rightMotor->run(FORWARD); //34, 86.5, 140 
    // leftMotor->run(FORWARD);
    // rightMotor->setSpeed(140);
    // leftMotor->setSpeed(144);

    // vTaskDelay(5000 / portTICK_PERIOD_MS);

    //initialize pid
    // for (int i = 0; i < 2; i++) {
    //     pid[i].setParams(0.25,0,0,255);
    // }
    pid[0].setParams(0.25,0,0,255);
    pid[1].setParams(0.25,0,0,255);

    motorTaskHandle = NULL;
    static uint8_t ucParameterToPass2; //parameter must exist for lifetime of task (so it is static)
    //xTaskCreate(motorTask, "motor", TASK_STACK_SIZE, &ucParameterToPass2, tskIDLE_PRIORITY+2, &motorTaskHandle); //lowest priority+2
    xTaskCreate(motorTask2_noEncoder, "motor", TASK_STACK_SIZE * 2, &ucParameterToPass2, tskIDLE_PRIORITY+2, &motorTaskHandle);
    //xTaskCreate(motorTask2, "motor", TASK_STACK_SIZE, &ucParameterToPass2, tskIDLE_PRIORITY+2, &motorTaskHandle);


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
    //xTaskCreate(imuTask, "imu", TASK_STACK_SIZE, &ucParameterToPass3, tskIDLE_PRIORITY+1, &imuTaskHandle); //lowest priority+1
    imuTaskHandle = NULL;
    static uint8_t ucParameterToPass3; 
    xTaskCreate(&mpu6050, "IMU", TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);
    //xTaskCreate(imu_task, "IMU", 4096, NULL, tskIDLE_PRIORITY+1, NULL);
    vTaskDelay(100);


    xTaskCreate(imu_motor_adjust, "IMU MOTOR", 2048, NULL, tskIDLE_PRIORITY+1, NULL);




    //----OTHER MISC STUFF----//

    // io_conf.pin_bit_mask = (1ULL << 13);
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // gpio_config(&io_conf);
    // int currVal = 0;
    // gpio_set_level(13, currVal);
    // vTaskDelay(5000 / portTICK_PERIOD_MS);


    //----WHILE LOOP----//


    // mpu6050_handle_t mpu6050_dev = mpu6050_create(I2C_MASTER_PORT, MPU6050_I2C_ADDRESS);
    // mpu6050_config(mpu6050_dev, ACCE_FS_4G, GYRO_FS_500DPS);
    // mpu6050_wake_up(mpu6050_dev);

    // mpu6050_acce_value_t acce;
    // mpu6050_gyro_value_t gyro;
    // complimentary_angle_t complimentary_angle;
    // while (1) {

    //     mpu6050_get_acce(mpu6050_dev, &acce);
    //     mpu6050_get_gyro(mpu6050_dev, &gyro);
    //     mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle);

    //     printf("x acceleration: %.3f, y acceleration: %.3f, z acceleration: %.3f, x rotation: %.3f, y rotation: %.3f, z rotation: %.3f\n", acce.acce_x, acce.acce_y, acce.acce_z, gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

    //     vTaskDelay(2000 / portTICK_PERIOD_MS);

    // }

    // long prevT = esp_timer_get_time(); //microseconds; used for time steps in pid control (derivative, etc)
    // long pos[2]; //every iteration read the motor pos into here so interrupts that update pos don't cause too much discrepancy

    // while (1) {
    //     //printf("%d\n", posiMotors[0]);
    //     long currT = esp_timer_get_time(); //get time in microseconds
    //     float deltaT = ((float) (currT - prevT))/( 1.0e6 ); //get difference in time between currT and prevT in seconds
    //     prevT = currT; //adjust prevT for next iteration

    //     //figure out what target position should be if some deltaT has passed based on some speed
    //     setTarget(currT/1.0e6, deltaT, 10); //currTime in seconds, deltaT in seconds, velocity (cm/s)

    //     //get curr motor positions (IDK IF I NEED TO DO THIS ATOMICALLY OR WITH LOCKS)
    //     for (int i = 0; i < 2; i++) {
    //         pos[i] = posiMotors[i];
    //     }

    //     //use pid controllers to figure out what control signals need to be sent to each motor based on curr motor positions and target positions
    //     for (int i = 0; i < 2; i++) {
    //         int dir, pwr; //dir = direction, pwr = value from 0-255 to set motor speed to
    //         // if (i == 0) {
    //         //     pid[i].evalu(pos[i], targetPosMotors[i], deltaT, pwr, dir, i); //figure out control signals
    //         // } else {
    //         //     pid[i].evalu(pos[i], pos[0] * -1, deltaT, pwr, dir, i); //figure out control signals
    //         // }
    //         pid[i].evalu(pos[i], targetPosMotors[i], deltaT, pwr, dir, i); //figure out control signals
    //         setMotor(motors[i], dir, pwr); //actually set motor speed and direction
    //         //printf("%d: curr pos: %ld, target pos: %ld, dir: %d, pwr: %d\n", i, pos[i], targetPosMotors[i], dir, pwr);
    //     }
    //     vTaskDelay(10 / portTICK_PERIOD_MS);
    // }

    while (1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    //ACTUAL CODE*********

    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // // rightMotor->run(FORWARD);
    // // leftMotor->run(FORWARD);
    // // rightMotor->setSpeed(90);
    // // leftMotor->setSpeed(90);

    // SimplePID anglePID;
    // anglePID.setParams(5,0,0,255);
    // long prevT = esp_timer_get_time(); //microseconds; used for time steps in pid control (derivative, etc)
    // long pos[2]; //every iteration read the motor pos into here so interrupts that update pos don't cause too much discrepancy
    // // float targetAngleDeg = 90;
    // // float targetAngleRad = targetAngleDeg * M_PI / 180.0; 
    // while (1) {

    //     long currT = esp_timer_get_time(); //get time in microseconds
    //     float deltaT = ((float) (currT - prevT))/( 1.0e6 ); //get difference in time between currT and prevT in seconds
    //     prevT = currT; //adjust prevT for next iteration


    //     //use pid controllers to figure out what control signals need to be sent to each motor based on curr motor positions and target positions
    //     int dir, pwr; //dir = direction, pwr = value from 0-255 to set motor speed to
    //     // anglePID.evaluAngle(global_yaw * (M_PI / 180), targetAngleRad, deltaT, pwr, dir); //figure out control signals
    //     // ESP_LOGI(TAG, "input yaw: %f, target: %f, deltaT: %f, pwr: %d, dir: %d", global_yaw * (M_PI / 180), targetAngleRad, deltaT, pwr, dir);
    //     anglePID.evaluAngle(global_yaw, targetAngleDeg, deltaT, pwr, dir); //figure out control signals
    //     ESP_LOGI(TAG, "input yaw: %f, target: %f, deltaT: %f, pwr: %d, dir: %d", global_yaw, targetAngleDeg, deltaT, pwr, dir);
    //     setMotor(motors[0], dir * -1, pwr); //actually set motor speed and direction
    //     setMotor(motors[1], dir, pwr); //actually set motor speed and direction
    //     // if (dir == 1) {
    //     //     int rPwr = -1 * pwr + 90;
    //     //     int rDir = 1;
    //     //     if (rPwr < 0) {
    //     //         rDir = -1;
    //     //         rPwr *= -1;
    //     //     }
    //     //     int lPwr = pwr + 90;
    //     //     if (lPwr > 255) {
    //     //         lPwr = 255;
    //     //     }
    //     //     setMotor(motors[0], rDir, rPwr); //actually set motor speed and direction
    //     //     setMotor(motors[1], 1, lPwr); //actually set motor speed and direction
    //     // } else {
    //     //     int lPwr = -1 * pwr + 90;
    //     //     int lDir = 1;
    //     //     if (lPwr < 0) {
    //     //         lDir = -1;
    //     //         lPwr *= -1;
    //     //     }
    //     //     int rPwr = pwr + 90;
    //     //     if (rPwr > 255) {
    //     //         rPwr = 255;
    //     //     }
    //     //     setMotor(motors[0], 1, rPwr); //actually set motor speed and direction
    //     //     setMotor(motors[1], lDir, lPwr); //actually set motor speed and direction

    //     // }

    //     vTaskDelay(50 / portTICK_PERIOD_MS);


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