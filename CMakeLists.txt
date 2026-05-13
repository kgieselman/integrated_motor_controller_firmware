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

set(CMAKE_PROJECT_NAME integrated_motor_controller_firmware)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C CXX ASM)

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
set(CUBEMX_DIR ${CMAKE_SOURCE_DIR}/cubemx)

# ---------------------------------------------------------------------------
# Compile definitions (mirrors cubemx/cmake/stm32cubemx/CMakeLists.txt)
# ---------------------------------------------------------------------------
set(MX_Defines_Syms
    USE_HAL_DRIVER
    STM32H563xx
    $<$<CONFIG:Debug>:DEBUG>
)

# ---------------------------------------------------------------------------
# Include paths
# ---------------------------------------------------------------------------
set(MX_Include_Dirs
    ${CUBEMX_DIR}/Core/Inc
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Inc
    ${CUBEMX_DIR}/Drivers/STM32H5xx_HAL_Driver/Inc/Legacy
    ${CUBEMX_DIR}/Drivers/CMSIS/Device/ST/STM32H5xx/Include
    ${CUBEMX_DIR}/Drivers/CMSIS/Include
)

# ---------------------------------------------------------------------------
# STM32 HAL + CMSIS driver sources (compiled as a static library)
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

# ---------------------------------------------------------------------------
# CubeMX peripheral init sources
# Excludes main.c — it is a reference only; main() lives in app/bringup/.
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
    ${CUBEMX_DIR}/Core/Src/stm32h5xx_it.c
    ${CUBEMX_DIR}/Core/Src/stm32h5xx_hal_msp.c
    ${CUBEMX_DIR}/Core/Src/sysmem.c
    ${CUBEMX_DIR}/Core/Src/syscalls.c
    ${CUBEMX_DIR}/startup_stm32h563xx.s
)

# ---------------------------------------------------------------------------
# Application sources — bringup
# ---------------------------------------------------------------------------
set(APP_Bringup_Src
    ${CMAKE_SOURCE_DIR}/app/bringup/src/main_bringup.cpp
)

# ---------------------------------------------------------------------------
# Targets
# ---------------------------------------------------------------------------
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_include_directories(STM32_Drivers PUBLIC ${MX_Include_Dirs})
target_compile_definitions(STM32_Drivers PUBLIC ${MX_Defines_Syms})

add_executable(${CMAKE_PROJECT_NAME})
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${MX_Periph_Src}
    ${APP_Bringup_Src}
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${MX_Include_Dirs}
    ${CMAKE_SOURCE_DIR}/app/bringup/include
)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ${MX_Defines_Syms}
)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    STM32_Drivers
    m
)

# Remove spurious libob.a pulled in by some gcc-arm-none-eabi versions
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Map file and memory usage report
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map
)
