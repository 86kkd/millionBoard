# 设置主程序源文件
set(COMPONENT_SRCS "main.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")

# 注册组件
register_component()

# 将组件添加到项目依赖中
set(COMPONENT_REQUIRES 
    motor_control_foc
    sensors
    communication
    battery_management
    utils
)