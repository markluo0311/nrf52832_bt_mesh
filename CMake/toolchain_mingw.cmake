# Generic Mesh CMake functionality
SET(CMAKE_SYSTEM_NAME Windows)

set(MINGW_DIR           "C:/mingw/bin")
set(CMAKE_AR            "${MINGW_DIR}/ar.exe" CACHE STRING "" FORCE)
set(CMAKE_C_COMPILER    "${MINGW_DIR}/gcc.exe")
set(CMAKE_C_LINKER      "${MINGW_DIR}/ld.exe")
set(CMAKE_RC_COMPILER   "${MINGW_DIR}/windres.exe")

set(CMAKE_C_FLAGS
    ${CMAKE_C_FLAGS}
    "-mno-ms-bitfields"
    )

string(REGEX REPLACE ";" " " CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")

set(CMAKE_C_LINK_FLAGS "--coverage" CACHE STRING "" FORCE)

function(set_properties_x86 target)
    if (NOT BUILD_HOST)
        message(FATAL_ERROR "Including x86 toolchain file for non-unit test build")
    endif (NOT BUILD_HOST)
    target_include_directories(${target} PRIVATE ${MBTLE_LOCAL_S130_HEADERS})
    target_include_directories(${target} PRIVATE ${MBTLE_LOCAL_TOOLCHAIN_HEADERS})

endfunction(set_properties_x86)
