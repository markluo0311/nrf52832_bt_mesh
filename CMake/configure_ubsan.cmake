# This script detects and enables supported address/undefined behaviour
# sanitizers when using GCC. Different versions of GCC has support for
# a different set of sanitizers.

if(CMAKE_HOST_UNIX)

    option(ENABLE_UBSAN "Enable the Undefined Behaviour Sanitizer" OFF)

    # Enable the Undefined Behaviour Sanitizer (ASAN and UBSAN libraries required)
    if(ENABLE_UBSAN)
        include(CheckCCompilerFlag)
        message(STATUS "Undefined behaviour sanitizer enabled")

        set(CMAKE_REQUIRED_LIBRARIES -lubsan -lasan) # Libraries required for the checking below

        set(UBSAN_COMPILER_FLAGS "-fno-sanitize-recover")
        mark_as_advanced(UBSAN_COMPILER_FLAGS)

        check_c_compiler_flag("-fsanitize=bounds" UBSAN_SANITIZE_BOUNDS_SUPPORTED)
        if(UBSAN_SANITIZE_BOUNDS_SUPPORTED)
            set(UBSAN_COMPILER_FLAGS "${UBSAN_COMPILER_FLAGS} -fsanitize=bounds")
            set(UBSAN_FEATURES_ENABLED "${UBSAN_FEATURES_ENABLED} +bounds")
        endif()

        check_c_compiler_flag("-fsanitize=address" UBSAN_SANITIZE_ADDRESS_SUPPORTED)
        if(UBSAN_SANITIZE_ADDRESS_SUPPORTED)
            set(UBSAN_COMPILER_FLAGS "${UBSAN_COMPILER_FLAGS} -fsanitize=address")
            set(UBSAN_FEATURES_ENABLED "${UBSAN_FEATURES_ENABLED} +address")
        endif()

        check_c_compiler_flag("-fsanitize=undefined" UBSAN_SANITIZE_UNDEFINED_SUPPORTED)
        if(UBSAN_SANITIZE_UNDEFINED_SUPPORTED)
            set(UBSAN_COMPILER_FLAGS "${UBSAN_COMPILER_FLAGS} -fsanitize=undefined")
            set(UBSAN_FEATURES_ENABLED "${UBSAN_FEATURES_ENABLED} +undefined")

            # If enabling the complete undefined behaviour sanitizer, we need to disable runtime alignment checking
            # because some unit tests rely on unaligned memory to work:
            check_c_compiler_flag("-fsanitize=undefined -fno-sanitize=alignment" UBSAN_NO_SANITIZE_ALIGNMENT_SUPPORTED)
            if(UBSAN_NO_SANITIZE_ALIGNMENT_SUPPORTED)
                set(UBSAN_COMPLER_FLAGS "${UBSAN_COMPILER_FLAGS} -fno-sanitize=alignment")
                set(UBSAN_FEATURES_ENABLED "${UBSAN_FEATURES_ENABLED} -alignment")
            endif()
        endif()

        message(STATUS "UBSAN/ASAN features: ${UBSAN_FEATURES_ENABLED}")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${UBSAN_COMPILER_FLAGS}")
        set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} ${UBSAN_COMPILER_FLAGS}")
    endif(ENABLE_UBSAN)

endif(CMAKE_HOST_UNIX)

