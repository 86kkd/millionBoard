# Battery management test component CMakeLists.txt
idf_component_register(
    SRCS "battery_management_test.c"
    INCLUDE_DIRS "."
    REQUIRES unity battery_management
) 