idf_component_register(SRCS "motor_task.c" "control_task.c" "rmt_step_test.c" "mpu6050.c" "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer
                    )