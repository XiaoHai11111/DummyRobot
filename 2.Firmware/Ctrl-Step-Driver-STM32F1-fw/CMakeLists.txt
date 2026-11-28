#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.20)

# specify cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(Ctrl-Step-STM32-fw C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
#add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
#add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(
        Core/Inc
        Drivers/STM32F1xx_HAL_Driver/Inc
        Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F1xx/Include
        Drivers/CMSIS/Include
        Ctrl
        Ctrl/Sensor
        Ctrl/Signal
        Ctrl/LowLevel
        Ctrl/MotorControl
        UserApp
        Port
)

add_definitions(-DUSE_HAL_DRIVER -D__MICROLIB -DSTM32F1 -DSTM32F1xx -DSTM32F103xB)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
        "startup/*.*"
        "Drivers/*.*"
        "Core/*.*"
        "Ctrl/*.*"
        "UserApp/*.*"
        "Port/*.*"
        )

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103CBTx_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m3 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

# The 20-kHz ISR has a 50-us deadline, including Debug builds. Keep symbols,
# but optimize every normal control/encoder/driver path without fast-math.
set_source_files_properties(
        Ctrl/Motor/motor.cpp Ctrl/Motor/motion_planner.cpp
        Ctrl/Sensor/Encoder/mt6816_base.cpp Ctrl/Driver/tb67h450_base.cpp
        Port/mt6816_stm32.cpp Port/tb67h450_stm32.cpp UserApp/main.cpp
        PROPERTIES COMPILE_OPTIONS "-O2;-fno-fast-math")

# CAN dispatch is interrupt driven rather than part of the 20 kHz control path.
# Prefer size here to keep the latest protocol additions inside the 95 KiB
# application flash window while retaining the real-time -O2 settings above.
set_source_files_properties(
        UserApp/protocols/interface_can.cpp
        PROPERTIES COMPILE_OPTIONS "-Os;-fno-fast-math;-ffunction-sections;-fdata-sections")
set_source_files_properties(
        UserApp/protocols/interface_uart.cpp
        Ctrl/Signal/button_base.cpp Ctrl/Signal/led_base.cpp
        Ctrl/Sensor/Encoder/encoder_calibrator_base.cpp
        Port/button_stm32.cpp Port/led_stm32.cpp
        Port/encoder_calibrator_stm32.cpp
        PROPERTIES COMPILE_OPTIONS "-Os;-fno-fast-math;-ffunction-sections;-fdata-sections")

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})
set_property(TARGET ${PROJECT_NAME}.elf APPEND PROPERTY LINK_DEPENDS ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
