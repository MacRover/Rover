# THIS FILE IS AUTOMATICALLY GENERATED. DO NOT EDIT.
# BASED ON c:\Users\Ethan\Documents\Repos\Rover\firmware\RAD

function(add_st_target_properties TARGET_NAME)

target_compile_definitions(
    ${TARGET_NAME} PRIVATE
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:ASM>>:DEBUG>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:DEBUG>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:USE_HAL_DRIVER>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:STM32F072xB>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:DEBUG>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:USE_HAL_DRIVER>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:STM32F072xB>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:USE_HAL_DRIVER>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:STM32F072xB>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:USE_HAL_DRIVER>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:STM32F072xB>"
)

target_include_directories(
    ${TARGET_NAME} PRIVATE
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Core\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F0xx_HAL_Driver\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F0xx_HAL_Driver\\Inc\\Legacy>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Device\\ST\\STM32F0xx\\Include>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Include>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Core\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F0xx_HAL_Driver\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F0xx_HAL_Driver\\Inc\\Legacy>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Device\\ST\\STM32F0xx\\Include>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Include>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Core\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F0xx_HAL_Driver\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F0xx_HAL_Driver\\Inc\\Legacy>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Device\\ST\\STM32F0xx\\Include>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Include>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Core\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F0xx_HAL_Driver\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F0xx_HAL_Driver\\Inc\\Legacy>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Device\\ST\\STM32F0xx\\Include>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Include>"
)

target_compile_options(
    ${TARGET_NAME} PRIVATE
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:ASM>>:-g3>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:-g3>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:-g3>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:ASM>>:-g0>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:-g0>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:-g0>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:-Os>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:-Os>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:>"
    "$<$<CONFIG:Debug>:-mcpu=cortex-m0>"
    "$<$<NOT:$<CONFIG:Debug>>:-mcpu=cortex-m0>"
)

target_link_libraries(
    ${TARGET_NAME} PRIVATE
)

target_link_directories(
    ${TARGET_NAME} PRIVATE
)

target_link_options(
    ${TARGET_NAME} PRIVATE
    "$<$<CONFIG:Debug>:-mcpu=cortex-m0>"
    "$<$<NOT:$<CONFIG:Debug>>:-mcpu=cortex-m0>"
    -T
    "$<$<CONFIG:Debug>:${PROJECT_SOURCE_DIR}/STM32F072RBTX_FLASH.ld>"
    "$<$<NOT:$<CONFIG:Debug>>:${PROJECT_SOURCE_DIR}/STM32F072RBTX_FLASH.ld>"
)

target_sources(
    ${TARGET_NAME} PRIVATE
    "Core\\Src\\main.c"
    "Core\\Src\\stm32f0xx_hal_msp.c"
    "Core\\Src\\stm32f0xx_it.c"
    "Core\\Src\\syscalls.c"
    "Core\\Src\\sysmem.c"
    "Core\\Src\\system_stm32f0xx.c"
    "Core\\Startup\\startup_stm32f072rbtx.s"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_can.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_cortex.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_dma.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_exti.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_flash_ex.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_flash.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_gpio.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_i2c_ex.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_i2c.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_pwr_ex.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_pwr.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_rcc_ex.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_rcc.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_spi_ex.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_spi.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_tim_ex.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal_tim.c"
    "Drivers\\STM32F0xx_HAL_Driver\\Src\\stm32f0xx_hal.c"
)

add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
)

add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
    $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.hex
)

add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
    $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.bin
)

endfunction()