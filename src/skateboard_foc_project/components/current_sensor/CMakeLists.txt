set(SRCS "src/current_sensor.c")
set(INCLUDE_DIRS "include")

# 设置依赖项
set(REQUIRES driver esp_adc motor_control_foc)

idf_component_register(
    SRCS "${SRCS}"
    INCLUDE_DIRS "${INCLUDE_DIRS}"
    REQUIRES "${REQUIRES}"
) 