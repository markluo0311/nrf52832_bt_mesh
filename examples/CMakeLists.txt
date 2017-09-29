# Example configuration
if(NRF_MESH_STACK_DEPTH)
    include_directories("${MBTLE_SOURCE_DIR}/mesh/test/include")
endif ()

macro(get_testname out_name)
    get_filename_component(testname ${CMAKE_CURRENT_LIST_DIR} NAME)
    set(${out_name} ${testname})
endmacro()

if(NOT BUILD_HOST)
    include_directories("hal/include")
    add_subdirectory(models)
    add_subdirectory(serial)
    add_subdirectory(beaconing)
    add_subdirectory(pb_remote_server)
    add_subdirectory(pb_remote_client)

    add_subdirectory(dfu)
    add_subdirectory(light_control)
endif()
