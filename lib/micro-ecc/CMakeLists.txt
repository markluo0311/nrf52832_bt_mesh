include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories("${MBTLE_SOURCE_DIR}/mesh/include/core")

add_library_platforms(UECC_LIB STATIC "uECC.c")

add_target_compile_definitions_platforms(UECC_LIB
    uECC_OPTIMIZATION_LEVEL=2
    #uECC_PLATFORM=uECC_arm_thumb
    uECC_SUPPORTS_secp160r1=0
    uECC_SUPPORTS_secp192r1=0
    uECC_SUPPORTS_secp224r1=0
    uECC_SUPPORTS_secp256r1=1
    uECC_SUPPORTS_secp256k1=0
    uECC_SUPPORT_COMPRESSED_POINT=0
    )

if (BUILD_TOOLCHAIN MATCHES "GCC")
    add_target_property_platforms(UECC_LIB COMPILE_FLAGS "-fomit-frame-pointer")
endif()
