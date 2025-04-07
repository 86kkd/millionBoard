# Communication test component CMakeLists.txt
idf_component_register(
    SRCS "communication_test.c"
    INCLUDE_DIRS "."
    REQUIRES unity communication
) 