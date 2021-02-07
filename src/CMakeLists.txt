cmake_minimum_required(VERSION 3.17)

PROJECT(TEMPLATE_SRC CXX)

#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
include("${CMAKE_CURRENT_LIST_DIR}/../modules.cmake/myconfig.cmake")
LOAD_MY_CONFIG()
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
# # main
include("${CMAKE_CURRENT_SOURCE_DIR}/../modules.cmake/project.cmake")
SET_TEMPLATE_VARIABLES()
CHECK_DIRECTORIES_EXIST(${TEMPLATE_SOURCE_DIR})
CHECK_DIRECTORIES_EXIST(${TEMPLATE_INCLUDE_DIR})
FIND_CPP(MY_TEMPLATE_SOURCES "${TEMPLATE_SOURCE_DIR}")
FIND_HPP(MY_TEMPLATE_HEADERS "${TEMPLATE_INCLUDE_DIR}")
list(APPEND MY_SOURCES ${MY_TEMPLATE_SOURCES})
list(APPEND MY_HEADERS ${MY_TEMPLATE_HEADERS})
DEFINE_ADD_LIBRARY(${TEMPLATE_LIB_NAME} "${MY_SOURCES}")
LOAD_INCLUDES_TEMPLATE_MODULE(${TEMPLATE_LIB_NAME})
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
## install
set_target_properties(
    ${TEMPLATE_LIB_NAME}
    PROPERTIES
    PUBLIC_HEADER "${MY_HEADERS}"
  )
install(
    TARGETS ${TEMPLATE_LIB_NAME}
    EXPORT ${TEMPLATE_LIB_NAME}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include
  )
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>