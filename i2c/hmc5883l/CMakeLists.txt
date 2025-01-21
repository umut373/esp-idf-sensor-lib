idf_component_register(SRCS "hmc5883l.cpp"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES i2c_dev calibrate)