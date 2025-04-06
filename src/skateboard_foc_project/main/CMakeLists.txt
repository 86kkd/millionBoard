# 设置主程序源文件
set(SRCS 
    "main.c"
)

# 设置包含目录
set(INCLUDE_DIRS ".")

# 注册组件
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES 
        motor_control_foc
        sensors
        communication
        battery_management
        utils
)