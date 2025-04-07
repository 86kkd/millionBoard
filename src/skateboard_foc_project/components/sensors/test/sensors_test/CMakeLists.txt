# Sensors test component CMakeLists.txt
idf_component_register(
    SRCS "sensors_test.c"
    INCLUDE_DIRS "."
    REQUIRES unity sensors
) 