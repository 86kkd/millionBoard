# 设置电机控制FOC组件配置
if(CONFIG_ENABLE_MOTOR_CONTROL_FOC)
    set(SRCS 
        "src/motor_control_foc.c"
    )

    # 设置包含目录
    set(INCLUDE_DIRS "include")
    
    set(COMPONENT_REQUIRES driver)
    
    # 只有当工具模块启用时才添加依赖
    if(CONFIG_ENABLE_UTILS)
        list(APPEND COMPONENT_REQUIRES utils)
    endif()
    
    # 只有当传感器模块启用时才添加依赖
    if(CONFIG_ENABLE_HX711 || CONFIG_ENABLE_ANGLE_SENSOR || CONFIG_ENABLE_NFC || CONFIG_ENABLE_GPS)
        list(APPEND COMPONENT_REQUIRES hx711 angle_sensor nfc gps)
    endif()

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
endif()