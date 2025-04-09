# 设置电池管理组件配置

set(SRCS 
    "src/battery_mgr.c"
)

# 设置包含目录
set(INCLUDE_DIRS "include")

# 注册组件
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES 
        driver
        communication
        utils
)

# 如果启用了单元测试，则添加测试目录
if(CONFIG_ENABLE_UNIT_TESTS)
    add_subdirectory(test)
endif()