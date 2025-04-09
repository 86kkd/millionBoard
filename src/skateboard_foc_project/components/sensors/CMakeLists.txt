# 设置传感器组件配置
if(CONFIG_ENABLE_SENSORS)
    set(COMPONENT_SRCS 
        "src/hx711.c"
        "src/angle_sensor.c"
        "src/nfc.c"
        "src/gps.c"
    )

    set(COMPONENT_ADD_INCLUDEDIRS "include")
    
    set(COMPONENT_REQUIRES driver esp_driver_gpio)
    
    # 只有当通信模块启用时才添加依赖
    if(CONFIG_ENABLE_COMMUNICATION)
        list(APPEND COMPONENT_REQUIRES communication)
    endif()
    
    # 只有当工具模块启用时才添加依赖
    if(CONFIG_ENABLE_UTILS)
        list(APPEND COMPONENT_REQUIRES utils)
    endif()

    # 注册组件
    idf_component_register(
        SRCS ${COMPONENT_SRCS}
        INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
        REQUIRES ${COMPONENT_REQUIRES}
        PRIV_REQUIRES esp_timer
    )

    # 添加测试目录
    if(CONFIG_ENABLE_UNIT_TESTS)
        add_subdirectory(test)
    endif()
endif()