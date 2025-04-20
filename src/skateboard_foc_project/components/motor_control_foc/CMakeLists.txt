# 设置电机控制FOC组件配置

set(SRCS 
    "src/motor_control_foc.cpp"
)

# 设置包含目录
set(INCLUDE_DIRS "include")

# 修复依赖项 - 确保指定正确的组件名
set(COMPONENT_REQUIRES 
    espressif__arduino-foc
    espressif__esp_simplefoc
    driver 
    esp_driver_gpio 
    current_sensor 
    esp_common 
    esp_adc 
)

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
