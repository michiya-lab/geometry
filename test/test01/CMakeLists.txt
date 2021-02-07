cmake_minimum_required(VERSION 3.17)

project(TEMPLATE_01 CXX)
set(TEST_NAME "UNIT_TEST_${PROJECT_NAME}")

file(GLOB SOURCE "*.cpp")
add_executable(${PROJECT_NAME} ${SOURCE})
#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
# include
include("${CMAKE_CURRENT_SOURCE_DIR}/../../modules.cmake/googletest.cmake")
LOAD_SETTING_GOOGLETEST_MODULE(${PROJECT_NAME})
include("${CMAKE_CURRENT_SOURCE_DIR}/../../modules.cmake/project.cmake")
LOAD_SETTING_TEMPLATE_MODULE(${PROJECT_NAME})
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

add_test(
    NAME ${TEST_NAME}
    COMMAND ${PROJECT_NAME} --gtest_output=xml:
)
set_tests_properties(${TEST_NAME} PROPERTIES TIMEOUT 1)