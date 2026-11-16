#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.19)

# specify cross compilers and tools. Preserve paths supplied by CLion/CMake;
# otherwise locate the STM32 GNU tools through PATH.
if(NOT CMAKE_C_COMPILER)
    find_program(CMAKE_C_COMPILER NAMES arm-none-eabi-gcc REQUIRED)
endif()
if(NOT CMAKE_CXX_COMPILER)
    find_program(CMAKE_CXX_COMPILER NAMES arm-none-eabi-g++ REQUIRED)
endif()
get_filename_component(ARM_TOOLCHAIN_BIN "${CMAKE_C_COMPILER}" DIRECTORY)
if(NOT CMAKE_ASM_COMPILER)
    set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}")
endif()
if(NOT CMAKE_AR)
    find_program(CMAKE_AR NAMES arm-none-eabi-ar HINTS "${ARM_TOOLCHAIN_BIN}" REQUIRED)
endif()
if(NOT CMAKE_OBJCOPY)
    find_program(CMAKE_OBJCOPY NAMES arm-none-eabi-objcopy HINTS "${ARM_TOOLCHAIN_BIN}" REQUIRED)
endif()
if(NOT CMAKE_OBJDUMP)
    find_program(CMAKE_OBJDUMP NAMES arm-none-eabi-objdump HINTS "${ARM_TOOLCHAIN_BIN}" REQUIRED)
endif()
if(NOT SIZE)
    find_program(SIZE NAMES arm-none-eabi-size HINTS "${ARM_TOOLCHAIN_BIN}" REQUIRED)
endif()
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(Core-STM32F4-fw C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# for use printf & scanf with float
set(COMMON_FLAGS "-specs=nosys.specs -specs=nano.specs -u _printf_float -u _scanf_float")

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
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
        Drivers/STM32F4xx_HAL_Driver/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/CMSIS/Include
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
        USB_DEVICE/App
        USB_DEVICE/Target
        3rdParty/fibre/cpp/include
        3rdParty/u8g2
        3rdParty/u8g2/cpp
        Bsp
        Bsp/imu
        Bsp/imu/filters
        Bsp/communication
        Bsp/memory
        Bsp/utils
        Bsp/gpio
        Bsp/utils/software_i2c
        Bsp/utils/arm_math
        Robot
        UserApp
)

add_definitions(-DUSE_HAL_DRIVER -DSTM32F4 -DSTM32F4xx -DSTM32F405xx -DconfigAPPLICATION_ALLOCATED_HEAP)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
        "startup/*.*"
        "Drivers/*.*"
        "Core/*.*"
        "UserApp/*.*"
        "3rdParty/*.*"
        "Middlewares/*.*"
        "USB_DEVICE/*.*"
        "Robot/*.*"
        "Bsp/*.*"
        )

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F405RGTx_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

link_directories("Drivers/CMSIS/Lib")
link_libraries("arm_cortexM4lf_math.a")

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

# Guard the USB/UART ASCII entry and reply frames against inlining regressions.
# This is a per-function bound, not a proof of the whole task's stack usage.
set_source_files_properties(
        UserApp/protocols/ascii_protocol.cpp Bsp/communication/ascii_processor.cpp
        PROPERTIES COMPILE_OPTIONS "-fstack-usage;-Werror=stack-usage=512")

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
