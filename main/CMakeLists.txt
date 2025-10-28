idf_component_register(SRCS "rmt_step_test.c" "mpu6050.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer
                    )