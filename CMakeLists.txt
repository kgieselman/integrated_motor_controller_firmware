cmake_minimum_required(VERSION 3.22)

# Language standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 20)
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
# CubeMX peripheral init — shared by every application target
# Note: stm32h5xx_hal_msp.c may need to diverge if a future target requires a
# different MSP configuration.
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
# stm32h5xx_it.c is target-specific and deliberately not listed here: tactical
# uses stm32h5xx_it_tactical.c instead, because the FreeRTOS port owns
# SVC_Handler, PendSV_Handler and SysTick_Handler. The CubeMX version is left
# in cubemx/ as generated reference; it is not compiled by any target.

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
# Library: imc_drivers
# Hand-written hardware drivers, shared by every application target.
#
# Every .cpp under drivers/ belongs to this list. A driver that is not listed
# is a driver nobody compiles — which is exactly how the stale pin references
# in Encoder and Drivetrain survived unnoticed. Add a file to drivers/, add it
# here in the same commit.
# ---------------------------------------------------------------------------
set(IMC_Drivers_Src
    ${CMAKE_SOURCE_DIR}/drivers/AdcDma.cpp
    ${CMAKE_SOURCE_DIR}/drivers/Battery.cpp
    ${CMAKE_SOURCE_DIR}/drivers/Buzzer.cpp
    ${CMAKE_SOURCE_DIR}/drivers/Console.cpp
    ${CMAKE_SOURCE_DIR}/drivers/CRSFReceiver.cpp
    ${CMAKE_SOURCE_DIR}/drivers/EEPROM.cpp
    ${CMAKE_SOURCE_DIR}/drivers/Encoder.cpp
    ${CMAKE_SOURCE_DIR}/drivers/IMU.cpp
    ${CMAKE_SOURCE_DIR}/drivers/Led.cpp
    ${CMAKE_SOURCE_DIR}/drivers/Motor.cpp
    ${CMAKE_SOURCE_DIR}/drivers/ServoChannel.cpp
)

# STATIC rather than OBJECT: -ffunction-sections, -fdata-sections and
# -Wl,--gc-sections are already set, so an executable pulls in only the
# drivers it actually references.
add_library(imc_drivers STATIC ${IMC_Drivers_Src})
target_include_directories(imc_drivers PUBLIC
    ${MX_Include_Dirs}
    ${CMAKE_SOURCE_DIR}/drivers
)
target_compile_definitions(imc_drivers PUBLIC ${MX_Defines_Syms})
target_link_libraries(imc_drivers PUBLIC STM32_Drivers)

# ---------------------------------------------------------------------------
# Target: imc_tactical
# Competition / mission control firmware
# ---------------------------------------------------------------------------
set(APP_Tactical_Src
    ${CMAKE_SOURCE_DIR}/app/tactical/main_tactical.cpp
    ${CMAKE_SOURCE_DIR}/app/tactical/subsystems/SubsystemManager.cpp
    ${CMAKE_SOURCE_DIR}/app/tactical/platform/InputSource.cpp
    ${CMAKE_SOURCE_DIR}/app/tactical/platform/SafetyMonitor.cpp
    ${CMAKE_SOURCE_DIR}/app/tactical/tasks/Tasks.cpp
    ${CMAKE_SOURCE_DIR}/app/tactical/tasks/CommsTask.cpp
    ${CMAKE_SOURCE_DIR}/app/tactical/tasks/ControlTask.cpp
    ${CMAKE_SOURCE_DIR}/app/tactical/tasks/TelemetryTask.cpp
    ${CMAKE_SOURCE_DIR}/app/tactical/tasks/HeartbeatTask.cpp
    ${CMAKE_SOURCE_DIR}/app/tactical/freertos_hooks.c
    ${CMAKE_SOURCE_DIR}/app/tactical/stm32h5xx_it_tactical.c
)

add_executable(imc_tactical)
target_sources(imc_tactical PRIVATE
    ${MX_Periph_Src}
    ${APP_Tactical_Src}
)
target_include_directories(imc_tactical PRIVATE
    ${MX_Include_Dirs}
    ${CMAKE_SOURCE_DIR}/app/tactical          # application headers, beside their sources
    ${CMAKE_SOURCE_DIR}/app/tactical/include  # FreeRTOSConfig.h only — a build artifact,
                                              # not an application header
)
target_compile_definitions(imc_tactical PRIVATE ${MX_Defines_Syms})
target_link_libraries(imc_tactical PRIVATE imc_drivers STM32_Drivers freertos_kernel m)
# -u _printf_float pulls in newlib's float formatting, which Console::printf
# needs for "%f". Without it a float prints as an empty field.
target_link_options(imc_tactical PRIVATE -Wl,-Map=${CMAKE_BINARY_DIR}/imc_tactical.map -u _printf_float)
set_target_properties(imc_tactical PROPERTIES
    ADDITIONAL_CLEAN_FILES ${CMAKE_BINARY_DIR}/imc_tactical.map
)
