# Utils test directory CMakeLists.txt
if(CONFIG_ENABLE_UNIT_TESTS)
    set(EXTRA_COMPONENT_DIRS ${EXTRA_COMPONENT_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
    set(TEST_COMPONENTS utils_test)
endif() 