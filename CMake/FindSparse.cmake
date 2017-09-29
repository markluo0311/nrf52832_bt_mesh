# Looks for the sparse command

find_program(SPARSE_EXECUTABLE sparse PATHS /usr/bin /usr/local/bin)
if(NOT SPARSE_EXECUTABLE)
    set(SPARSE_FOUND FALSE)
else()
    message("-- Sparse found at ${SPARSE_EXECUTABLE}")
    set(SPARSE_FOUND TRUE)

    # Find the GCC include directory:
    execute_process(COMMAND ${CMAKE_C_COMPILER} -print-file-name=include OUTPUT_VARIABLE GCC_INTERNAL_INCLUDE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
    message("-- Found GCC headers in ${GCC_INTERNAL_INCLUDE_DIR}")
    set(GCC_INCLUDE_DIR "${GCC_INTERNAL_INCLUDE_DIR}/..")

    set(SPARSE_FLAGS -Wsparse-all -Wno-declaration-after-statement CACHE STRING "Command line arguments for the sparse command")

    macro(enable_sparse directory sourcedir include_dirs)
        # Create a list of defines:
        get_directory_property(SPARSE_DEFINE_LIST DIRECTORY ${directory} COMPILE_DEFINITIONS)
        foreach(DEFINE ${SPARSE_DEFINE_LIST})
            set(SPARSE_DEFINES ${SPARSE_DEFINES} -D${DEFINE})
        endforeach()
        set(SPARSE_DEFINES ${SPARSE_DEFINES} -DSVCALL_AS_NORMAL_FUNCTION)

        # Create a list of include directories:
        get_directory_property(SPARSE_INCLUDE_LIST DIRECTORY ${directory} INCLUDE_DIRECTORIES)
        foreach(INCDIR ${SPARSE_INCLUDE_LIST})
            set(SPARSE_INCLUDES ${SPARSE_INCLUDES} -I${INCDIR})
        endforeach()
        foreach(INCDIR ${include_dirs})
            set(SPARSE_INCLUDES ${SPARSE_INCLUDES} -I${INCDIR})
        endforeach()

        # Add the command for running sparse:
        add_custom_target(sparse
            sparse -gcc-base-dir ${GCC_INCLUDE_DIR}
            ${SPARSE_FLAGS}
            ${SPARSE_INCLUDES}
            ${SPARSE_DEFINES}
            ${sourcedir}/*.c)
    endmacro()

endif()
