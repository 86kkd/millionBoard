# 设置主程序源文件
set(COMPONENT_SRCS 
    "main.c"
    "freertos_hooks.c"
)
set(COMPONENT_ADD_INCLUDEDIRS ".")

# 注册组件
idf_component_register(
    SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
    REQUIRES motor_control_foc sensors communication battery_management utils nvs_flash
)