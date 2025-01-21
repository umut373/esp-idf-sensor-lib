idf_component_register(SRCS "bno055.cpp"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES i2c_dev calibrate)