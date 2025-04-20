# 设置主程序源文件
set(COMPONENT_SRCS 
    "main.c"
    "freertos_hooks.c"
)
set(COMPONENT_ADD_INCLUDEDIRS ".")

set(REQUIRES_LIST 
    motor_control_foc
    communication
    battery_management
    utils
    driver
    pressure_sensor
    angle_sensor
    nfc
    gps
    nvs_flash
)
# 注册组件
idf_component_register(
    SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
    REQUIRES ${REQUIRES_LIST}
)