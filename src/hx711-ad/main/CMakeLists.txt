set(srcs "main.cpp" "scale.cpp")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "."
)
