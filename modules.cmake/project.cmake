#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
set_property(GLOBAL PROPERTY PATH_TEMPLATE_MODULE "${CMAKE_CURRENT_LIST_DIR}")
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
macro(SET_TEMPLATE_VARIABLES)
    get_property(MY_MODULE_PATH GLOBAL PROPERTY PATH_TEMPLATE_MODULE)
    if(NOT DEFINED TEMPLATE_LIB_NAME)
        set(TEMPLATE_LIB_NAME "michiya_TEMPLATE")
        get_property(
            MY_MODULE_PATH
            GLOBAL PROPERTY
            PATH_TEMPLATE_MODULE
            )
        set(
            TEMPLATE_SOURCE_DIR
            "${MY_MODULE_PATH}/../src"
            )
        set(
            TEMPLATE_INCLUDE_DIR
            "/opt/eigen-3.3.9"
            "${MY_MODULE_PATH}/../src"
            )
    endif()
endmacro()
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
macro(LOAD_DEPENDENCIES_TEMPLATE_MODULE MACRO_ARG)
    SET_TEMPLATE_VARIABLES()
    add_dependencies(${MACRO_ARG} ${TEMPLATE_LIB_NAME})
endmacro()
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
macro(LOAD_LINKS_TEMPLATE_MODULE MACRO_ARG)
    SET_TEMPLATE_VARIABLES()
    target_link_libraries(
        ${MACRO_ARG}
        PRIVATE ${TEMPLATE_LIB_NAME}
        )
endmacro()
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
macro(LOAD_INCLUDES_TEMPLATE_MODULE MACRO_ARG)
    SET_TEMPLATE_VARIABLES()
    target_include_directories(
        ${MACRO_ARG}
        PRIVATE ${TEMPLATE_INCLUDE_DIR}
    )
endmacro()
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
macro(LOAD_SETTING_TEMPLATE_MODULE MACRO_ARG)
    LOAD_DEPENDENCIES_TEMPLATE_MODULE(${MACRO_ARG})
    LOAD_LINKS_TEMPLATE_MODULE(${MACRO_ARG})
    LOAD_INCLUDES_TEMPLATE_MODULE(${MACRO_ARG})
endmacro()
#<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<