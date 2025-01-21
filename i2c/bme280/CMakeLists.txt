idf_component_register(SRCS "bme280.cpp"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES i2c_dev)