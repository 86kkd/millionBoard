# Utils test component CMakeLists.txt
idf_component_register(
    SRCS "utils_test.c"
    INCLUDE_DIRS "."
    REQUIRES unity utils
) 