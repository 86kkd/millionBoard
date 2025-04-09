# 设置主程序源文件
set(COMPONENT_SRCS 
    "main.c"
    "freertos_hooks.c"
)
set(COMPONENT_ADD_INCLUDEDIRS ".")

set(REQUIRES_LIST "")

# 根据 Kconfig 设置添加依赖
if(CONFIG_ENABLE_MOTOR_CONTROL_FOC)
    list(APPEND REQUIRES_LIST motor_control_foc)
endif()

if(CONFIG_ENABLE_SENSORS)
    list(APPEND REQUIRES_LIST sensors)
endif()

if(CONFIG_ENABLE_COMMUNICATION)
    list(APPEND REQUIRES_LIST communication)
endif()

if(CONFIG_ENABLE_BATTERY_MANAGEMENT)
    list(APPEND REQUIRES_LIST battery_management)
endif()

if(CONFIG_ENABLE_UTILS)
    list(APPEND REQUIRES_LIST utils)
endif()

# 添加基本依赖项
list(APPEND REQUIRES_LIST driver)

# 注册组件
idf_component_register(
    SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
    REQUIRES ${REQUIRES_LIST}
)