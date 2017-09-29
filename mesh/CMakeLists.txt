# Set provisioning flags dependeing on settings:
set(BUILD_PERSISTENT_STORAGE_LIB OFF CACHE STRING "Build core library with persistent storage?")
mark_as_advanced(BUILD_PERSISTENT_STORAGE_LIB)

include_directories(
    "${MBTLE_LIB_HEADERS}"
    "${MBTLE_SRC_HEADERS}"
    "${MBTLE_HOST_HEADERS}"
    )

add_definitions(${MBTLE_DEFINITIONS})

if(BUILD_HOST)
    add_subdirectory("test")
else()
    message (STATUS "MBTLE_DEFINITIONS = ${MBTLE_DEFINITIONS}")

    if (PERSISTENT_STORAGE)
        add_library_platforms(MBTLE_CORE STATIC ${WEAK} ${MBTLE_SRCS_CORE} ${MBTLE_SRCS_FLASH})
        add_target_compile_definitions_platforms(MBTLE_CORE "PERSISTENT_STORAGE=1")
    else()
        add_library_platforms(MBTLE_CORE STATIC ${WEAK} ${MBTLE_SRCS_CORE})
        add_target_compile_definitions_platforms(MBTLE_CORE "PERSISTENT_STORAGE=0")
    endif()

    if (BUILD_PERSISTENT_STORAGE_LIB)
        add_library_platforms(MBTLE_CORE_FLASH STATIC ${WEAK} ${MBTLE_SRCS_CORE} ${MBTLE_SRCS_FLASH})
        add_target_compile_definitions_platforms(MBTLE_CORE_FLASH "PERSISTENT_STORAGE=1")
    endif()

    add_library_platforms(MBTLE_PB_ADV_EE  STATIC ${MBTLE_SRCS_PROV} ${MBTLE_SRCS_PB_ADV_EE})
    add_library_platforms(MBTLE_PB_ADV_ER  STATIC ${MBTLE_SRCS_PROV} ${MBTLE_SRCS_PB_ADV_ER})

    #add_library_platforms(MBTLE_SERIAL     STATIC ${MBTLE_SRCS_SERIAL})

endif()

find_package(Sparse)
if(NOT SPARSE_FOUND)
    message("-- Sparse not found, sparse checking disabled")
else()
    message("-- Sparse checking enabled")
    enable_sparse(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src "include/;api/")
endif()
