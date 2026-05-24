cmake_minimum_required(VERSION 3.22)

# Language standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(integrated_motor_controller)
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C CXX ASM)

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
set(CUBEMX_DIR ${CMAKE_SOURCE_DIR}/cubemx)

# ---------------------------------------------------------------------------
# Compile definitions
# ---------------------------------------------------------------------------
set(MX_Defines_Syms
    USE_HAL_DRIVER
    STM32H563xx
    $<$<CONFIG:Debug>:DEBUG>
)

# ---------------------------------------------------------------------------
# Include paths (HAL + CMSIS)
# ---------------------------------------------------------------------------
set(MX_Include_Dirs
    ${CUBEMX_DIR}/Core/Inc
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Inc
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy
    ${CUBEMX_DIR}/Drivers/CMSIS/Device/ST/STM32H5xx/Include
    ${CUBEMX_DIR}/Drivers/CMSIS/Include
)

# ---------------------------------------------------------------------------
# STM32 HAL + CMSIS — compiled once, shared by both targets
# ---------------------------------------------------------------------------
set(STM32_Drivers_Src
    ${CUBEMX_DIR}/Core/Src/system_stm32h5xx.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_adc.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_adc_ex.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_cortex.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_dma_ex.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_exti.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash_ex.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_gpio.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_i2c.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_i2c_ex.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_icache.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pcd.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pcd_ex.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_pwr_ex.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_rcc_ex.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_spi.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_spi_ex.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_tim.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_tim_ex.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_uart.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_uart_ex.c
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_ll_usb.c
)

add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_include_directories(STM32_Drivers PUBLIC ${MX_Include_Dirs})
target_compile_definitions(STM32_Drivers PUBLIC ${MX_Defines_Syms})

# ---------------------------------------------------------------------------
# CubeMX peripheral init — shared across both targets
# Note: stm32h5xx_it.c and stm32h5xx_hal_msp.c may need to diverge if
# bringup and tactical require different IRQ or MSP configurations.
# ---------------------------------------------------------------------------
set(MX_Periph_Src
    ${CUBEMX_DIR}/Core/Src/gpio.c
    ${CUBEMX_DIR}/Core/Src/adc.c
    ${CUBEMX_DIR}/Core/Src/gpdma.c
    ${CUBEMX_DIR}/Core/Src/i2c.c
    ${CUBEMX_DIR}/Core/Src/icache.c
    ${CUBEMX_DIR}/Core/Src/spi.c
    ${CUBEMX_DIR}/Core/Src/tim.c
    ${CUBEMX_DIR}/Core/Src/usart.c
    ${CUBEMX_DIR}/Core/Src/usb.c
    ${CUBEMX_DIR}/Core/Src/stm32h5xx_hal_msp.c
    ${CUBEMX_DIR}/Core/Src/sysmem.c
    ${CUBEMX_DIR}/Core/Src/syscalls.c
    ${CUBEMX_DIR}/startup_stm32h563xx.s
)
# stm32h5xx_it.c is target-specific: bringup uses the CubeMX version,
# tactical uses stm32h5xx_it_tactical.c (FreeRTOS owns SVC/PendSV/SysTick).

# Remove spurious libob.a pulled in by some gcc-arm-none-eabi versions
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ---------------------------------------------------------------------------
# FreeRTOS Kernel — fetched at configure time, used by imc_tactical only
# ---------------------------------------------------------------------------
include(FetchContent)

FetchContent_Declare(
    freertos_kernel
    GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
    GIT_TAG        V11.1.0
    GIT_SHALLOW    TRUE
)

# 'freertos_config' must exist before FetchContent_MakeAvailable — FreeRTOS
# CMakeLists checks for it with if(NOT TARGET freertos_config) at configure time.
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM INTERFACE
    "${CMAKE_SOURCE_DIR}/app/tactical/include"
)

set(FREERTOS_PORT GCC_ARM_CM33_NTZ_NONSECURE CACHE STRING "FreeRTOS port" FORCE)
set(FREERTOS_HEAP 4                           CACHE STRING "FreeRTOS heap implementation" FORCE)

FetchContent_MakeAvailable(freertos_kernel)

# ---------------------------------------------------------------------------
# Target: imc_bringup
# Interactive USB-CDC console for hardware validation (11-stage checklist)
# ---------------------------------------------------------------------------
set(APP_Bringup_Src
    ${CMAKE_SOURCE_DIR}/app/bringup/src/main_bringup.cpp
    ${CMAKE_SOURCE_DIR}/app/bringup/src/Console.cpp
    ${CMAKE_SOURCE_DIR}/app/bringup/src/tests/BuzzerTest.cpp
    ${CMAKE_SOURCE_DIR}/app/bringup/src/tests/ImuTest.cpp
    ${CMAKE_SOURCE_DIR}/app/bringup/src/tests/MotorTest.cpp
    ${CMAKE_SOURCE_DIR}/app/bringup/src/tests/EncoderTest.cpp
    ${CMAKE_SOURCE_DIR}/app/bringup/src/tests/BatteryTest.cpp
    ${CMAKE_SOURCE_DIR}/app/bringup/src/tests/EepromTest.cpp
    ${CMAKE_SOURCE_DIR}/drivers/src/EEPROM.cpp
    ${CMAKE_SOURCE_DIR}/drivers/src/IMU.cpp
    ${CMAKE_SOURCE_DIR}/drivers/src/ServoChannel.cpp
    ${CMAKE_SOURCE_DIR}/app/bringup/src/tests/ServoTest.cpp
    ${CMAKE_SOURCE_DIR}/app/bringup/src/tests/CrsfTest.cpp
    ${CMAKE_SOURCE_DIR}/app/bringup/src/tests/LedTest.cpp
)

add_executable(imc_bringup)
target_sources(imc_bringup PRIVATE
    ${MX_Periph_Src}
    ${CUBEMX_DIR}/Core/Src/stm32h5xx_it.c
    ${APP_Bringup_Src}
)
target_include_directories(imc_bringup PRIVATE
    ${MX_Include_Dirs}
    ${CMAKE_SOURCE_DIR}/app/bringup/include
    ${CMAKE_SOURCE_DIR}/drivers/include
)
target_compile_definitions(imc_bringup PRIVATE ${MX_Defines_Syms})
target_link_libraries(imc_bringup PRIVATE STM32_Drivers m)
target_link_options(imc_bringup PRIVATE -Wl,-Map=${CMAKE_BINARY_DIR}/imc_bringup.map -u _printf_float)
set_target_properties(imc_bringup PROPERTIES
    ADDITIONAL_CLEAN_FILES ${CMAKE_BINARY_DIR}/imc_bringup.map
)

# ---------------------------------------------------------------------------
# Target: imc_tactical
# Competition / mission control firmware (FreeRTOS to be added)
# ---------------------------------------------------------------------------
set(APP_Tactical_Src
    ${CMAKE_SOURCE_DIR}/app/tactical/src/main_tactical.cpp
    ${CMAKE_SOURCE_DIR}/app/tactical/src/freertos_hooks.c
    ${CMAKE_SOURCE_DIR}/app/tactical/src/stm32h5xx_it_tactical.c
)

add_executable(imc_tactical)
target_sources(imc_tactical PRIVATE
    ${MX_Periph_Src}
    ${APP_Tactical_Src}
)
target_include_directories(imc_tactical PRIVATE
    ${MX_Include_Dirs}
    ${CMAKE_SOURCE_DIR}/app/tactical/include
)
target_compile_definitions(imc_tactical PRIVATE ${MX_Defines_Syms})
target_link_libraries(imc_tactical PRIVATE STM32_Drivers freertos_kernel m)
target_link_options(imc_tactical PRIVATE -Wl,-Map=${CMAKE_BINARY_DIR}/imc_tactical.map)
set_target_properties(imc_tactical PROPERTIES
    ADDITIONAL_CLEAN_FILES ${CMAKE_BINARY_DIR}/imc_tactical.map
)
