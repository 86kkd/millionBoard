# 集成测试配置
if(CONFIG_ENABLE_UNIT_TESTS)
    set(SRCS 
        "integration/test_system.c"
    )

    # 设置包含目录
    set(INCLUDE_DIRS ".")

    # 注册测试组件
    idf_component_register(
        SRCS ${SRCS}
        INCLUDE_DIRS ${INCLUDE_DIRS}
        REQUIRES
            unity
            motor_control_foc
            sensors
            communication
            battery_management
            utils
    )
endif()
