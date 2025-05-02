set(srcs "main.cpp" "pressure_sensor.cpp")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "."
)
