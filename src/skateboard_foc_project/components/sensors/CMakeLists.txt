# 设置传感器组件配置
set(COMPONENT_SRCS 
    "src/hx711.c"
    "src/angle_sensor.c"
    "src/nfc.c"
    "src/gps.c"
)

set(COMPONENT_ADD_INCLUDEDIRS "include")

# 注册组件
idf_component_register(
    SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
    REQUIRES driver communication utils esp_driver_gpio
    PRIV_REQUIRES esp_timer
)

# 添加测试目录
if(CONFIG_ENABLE_UNIT_TESTS)
    add_subdirectory(test)
endif()