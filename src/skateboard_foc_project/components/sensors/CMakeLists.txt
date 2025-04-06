# 设置传感器组件配置
set(COMPONENT_SRCS 
    "src/hx711.c"
    "src/angle_sensor.c"
    "src/nfc.c"
    "src/gps.c"
)

set(COMPONENT_ADD_INCLUDEDIRS "include")

# 注册组件
register_component()

# 设置组件依赖
set(COMPONENT_REQUIRES 
    driver
    communication
    utils
)

# 添加测试目录
if(CONFIG_ENABLE_UNIT_TESTS)
    add_subdirectory(test)
endif()