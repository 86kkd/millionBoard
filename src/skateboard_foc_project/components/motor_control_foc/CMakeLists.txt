# 设置电机控制FOC组件配置

set(SRCS 
    "src/motor_control_foc.c"
)

# 设置包含目录
set(INCLUDE_DIRS "include")

# 根据需要添加依赖项
set(COMPONENT_REQUIRES driver esp_driver_gpio current_sensor)


# 注册组件
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES ${COMPONENT_REQUIRES}
)

# 如果启用了单元测试，则添加测试目录
if(CONFIG_ENABLE_UNIT_TESTS)
    add_subdirectory(test)
endif()
