idf_component_register(SRCS "main.cpp" "SimplePID.cpp" "mpu6050_DMP.cpp"
                    INCLUDE_DIRS "." "include"
                    REQUIRES ultrasonic
                    REQUIRES I2Cdev
                    REQUIRES AK8963
                    REQUIRES KalmanFilter
                    REQUIRES MPU6050
                    REQUIRES MadgwickAHRS
                    # REQUIRES mpu9250
                    # REQUIRES ahrs
                    REQUIRES AFMSv2ESP32)
