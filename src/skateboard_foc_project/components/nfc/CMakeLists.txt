# NFC 组件配置

set(SRCS 
    "src/nfc.c"
)

set(INCLUDE_DIRS "include")

set(COMPONENT_REQUIRES driver rc522)

# 根据需要添加依赖项
if(CONFIG_ENABLE_UTILS)
    list(APPEND COMPONENT_REQUIRES utils )
endif()

# 注册组件
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES ${COMPONENT_REQUIRES}
)

# 添加测试目录
if(CONFIG_ENABLE_UNIT_TESTS)
    add_subdirectory(test)
endif()

