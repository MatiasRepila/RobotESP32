idf_component_register(
  SRCS "main.c" "control_task.c" "mpu6050.c" "bt_spp.c" "motor_timer_dual.c"
  INCLUDE_DIRS "."
  REQUIRES driver nvs_flash bt
)
