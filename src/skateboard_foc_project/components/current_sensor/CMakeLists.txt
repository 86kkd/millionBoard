set(SRCS "src/current_sensor.cpp")
set(INCLUDE_DIRS "include")

# 设置依赖项
set(REQUIRES driver esp_adc esp_simplefoc)

idf_component_register(
    SRCS "${SRCS}"
    INCLUDE_DIRS "${INCLUDE_DIRS}"
    REQUIRES "${REQUIRES}"
) 