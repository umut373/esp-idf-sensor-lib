idf_component_register(SRCS "bmp180.cpp"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES i2c_dev calibrate)