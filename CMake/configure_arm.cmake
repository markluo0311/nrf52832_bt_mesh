# Setup for compiling on target (arm)
include("CMake/configure_hw.cmake")

set(NRF51_C_FLAGS
    "NRF51"
    "__HEAP_SIZE=128"
    "NRF51422"
    "${NRF51_SOFTDEVICE}"
    "NRF_SD_BLE_API_VERSION=2"
    )

set(NRF52_C_FLAGS
    "NRF52"
    "${NRF52_SOFTDEVICE}"
    "__HEAP_SIZE=1024"
    "NRF52832"
    "NRF52_PAN_1"
    "NRF52_PAN_2"
    "NRF52_PAN_3"
    "NRF52_PAN_4"
    "NRF52_PAN_7"
    "NRF52_PAN_8"
    "NRF52_PAN_9"
    "NRF52_PAN_10"
    "NRF52_PAN_11"
    "NRF52_PAN_12"
    "NRF52_PAN_15"
    "NRF52_PAN_16"
    "NRF52_PAN_17"
    "NRF52_PAN_20"
    "NRF52_PAN_23"
    "NRF52_PAN_24"
    "NRF52_PAN_25"
    "NRF52_PAN_26"
    "NRF52_PAN_27"
    "NRF52_PAN_28"
    "NRF52_PAN_29"
    "NRF52_PAN_30"
    "NRF52_PAN_32"
    "NRF52_PAN_33"
    "NRF52_PAN_34"
    "NRF52_PAN_35"
    "NRF52_PAN_36"
    "NRF52_PAN_37"
    "NRF52_PAN_38"
    "NRF52_PAN_39"
    "NRF52_PAN_40"
    "NRF52_PAN_41"
    "NRF52_PAN_42"
    "NRF52_PAN_43"
    "NRF52_PAN_44"
    "NRF52_PAN_46"
    "NRF52_PAN_47"
    "NRF52_PAN_48"
    "NRF52_PAN_49"
    "NRF52_PAN_58"
    "NRF52_PAN_63"
    "NRF52_PAN_64"
    "NRF52_PAN_65"
    "CONFIG_GPIO_AS_PINRESET"
    "NRF_SD_BLE_API_VERSION=3"
    )

if (BUILD_TOOLCHAIN MATCHES "GCC")
    include("CMake/toolchain_gcc.cmake")

    function(create_hex_platform executable)
        foreach(id ${HWID})
            add_custom_command(
                TARGET ${id}_${executable}
                POST_BUILD
                COMMAND arm-none-eabi-objcopy -O ihex ${CMAKE_CURRENT_BINARY_DIR}/${id}_${executable}.elf ${CMAKE_CURRENT_BINARY_DIR}/${id}_${executable}.hex
                )
        endforeach()
    endfunction(create_hex_platform)

    function(create_hex executable)
        add_custom_command(
            TARGET ${executable}
            POST_BUILD
            COMMAND arm-none-eabi-objcopy -O ihex ${CMAKE_CURRENT_BINARY_DIR}/${executable}.elf ${CMAKE_CURRENT_BINARY_DIR}/${executable}.hex
            )
    endfunction(create_hex)

elseif(BUILD_TOOLCHAIN MATCHES "ARM")
    include("CMake/toolchain_armcc.cmake")

    function(create_hex_platform executable)
        foreach(id ${HWID})
            add_custom_command(
                TARGET ${id}_${executable}
                POST_BUILD
                COMMAND ${ARMCC_FROMELF} --i32combined -o ${CMAKE_CURRENT_BINARY_DIR}/${id}_${executable}.hex ${CMAKE_CURRENT_BINARY_DIR}/${id}_${executable}.elf
                )
        endforeach()
    endfunction(create_hex_platform)

    function(create_hex executable)
        add_custom_command(
            TARGET ${executable}
            POST_BUILD
            COMMAND ${ARMCC_FROMELF} --i32combined -o ${CMAKE_CURRENT_BINARY_DIR}/${executable}.hex ${CMAKE_CURRENT_BINARY_DIR}/${executable}.elf
            )
    endfunction(create_hex)

else ()
    message(FATAL_ERROR "No compiler toolchain selected. Valid options are \"ARM\" and \"GCC\"")
endif ()


function (add_flash_target target)
    foreach(id ${HWID})
        set(f_target f_${id}_${target})
        if (id STREQUAL "51" AND NRF51_SOFTDEVICE STREQUAL "S110")
            set(softdevice_path "${CMAKE_SOURCE_DIR}/lib/softdevice/s110/hex/s110_softdevice.hex")
            set(softdevice_version "s110")
        elseif (id STREQUAL "51" AND NRF51_SOFTDEVICE STREQUAL "S130")
            set(softdevice_path "${CMAKE_SOURCE_DIR}/lib/softdevice/s130/hex/s130_nrf51_2.0.1_softdevice.hex")
            set(softdevice_version "s130")
        elseif (id STREQUAL "52" AND NRF52_SOFTDEVICE STREQUAL "S132")
            set(softdevice_path "${CMAKE_SOURCE_DIR}/lib/softdevice/s132/hex/s132_nrf52_3.0.0_softdevice.hex")
            set(softdevice_version "s132")
        else ()
            message(FATAL_ERROR "Adding flash target for unsupported platform: " ${id} ${NRF51_SOFTDEVICE} ${NRF52_SOFTDEVICE})
        endif()
        add_custom_target(${f_target}
            COMMAND mergehex -m ${CMAKE_CURRENT_BINARY_DIR}/${id}_${target}.hex ${softdevice_path} -o ${CMAKE_CURRENT_BINARY_DIR}/${id}_${target}_${softdevice_version}.hex
            COMMAND nrfjprog -f NRF${id} --eraseall
            COMMAND nrfjprog -f NRF${id} --program ${CMAKE_CURRENT_BINARY_DIR}/${id}_${target}_${softdevice_version}.hex
            DEPENDS ${id}_${target}
            VERBATIM
            )
    endforeach()
endfunction()
