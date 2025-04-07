# Motor control FOC test component CMakeLists.txt
idf_component_register(
    SRCS "motor_control_foc_test.c"
    INCLUDE_DIRS "."
    REQUIRES unity motor_control_foc
) 