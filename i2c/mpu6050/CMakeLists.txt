idf_component_register(SRCS "mpu6050.cpp"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES i2c_dev calibrate esp_timer)