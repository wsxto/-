# groups.cmake

# group Application/MDK-ARM
add_library(Group_Application_MDK-ARM OBJECT
  "${SOLUTION_ROOT}/startup_stm32h723xx.s"
)
target_include_directories(Group_Application_MDK-ARM PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_MDK-ARM PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_MDK-ARM_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Application_MDK-ARM_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Application_MDK-ARM PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_MDK-ARM PUBLIC
  Group_Application_MDK-ARM_ABSTRACTIONS
)
set(COMPILE_DEFINITIONS
  STM32H723xx
  _RTE_
)
cbuild_set_defines(AS_ARM COMPILE_DEFINITIONS)
set_source_files_properties("${SOLUTION_ROOT}/startup_stm32h723xx.s" PROPERTIES
  COMPILE_FLAGS "${COMPILE_DEFINITIONS}"
)

# group Application/User/Core
add_library(Group_Application_User_Core OBJECT
  "${SOLUTION_ROOT}/../Core/Src/main.c"
  "${SOLUTION_ROOT}/../Core/Src/gpio.c"
  "${SOLUTION_ROOT}/../Core/Src/memorymap.c"
  "${SOLUTION_ROOT}/../Core/Src/stm32h7xx_it.c"
  "${SOLUTION_ROOT}/../Core/Src/stm32h7xx_hal_msp.c"
  "${SOLUTION_ROOT}/../Core/Src/stm32h7xx_hal_timebase_tim.c"
)
target_include_directories(Group_Application_User_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_User_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_User_Core_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Application_User_Core_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Application_User_Core PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_User_Core PUBLIC
  Group_Application_User_Core_ABSTRACTIONS
)

# group Application/User/USB_DEVICE/App
add_library(Group_Application_User_USB_DEVICE_App OBJECT
  "${SOLUTION_ROOT}/../USB_DEVICE/App/usb_device.c"
  "${SOLUTION_ROOT}/../USB_DEVICE/App/usbd_desc.c"
  "${SOLUTION_ROOT}/../USB_DEVICE/App/usbd_cdc_if.c"
)
target_include_directories(Group_Application_User_USB_DEVICE_App PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_User_USB_DEVICE_App PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_User_USB_DEVICE_App_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Application_User_USB_DEVICE_App_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Application_User_USB_DEVICE_App PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_User_USB_DEVICE_App PUBLIC
  Group_Application_User_USB_DEVICE_App_ABSTRACTIONS
)

# group Application/User/USB_DEVICE/Target
add_library(Group_Application_User_USB_DEVICE_Target OBJECT
  "${SOLUTION_ROOT}/../USB_DEVICE/Target/usbd_conf.c"
)
target_include_directories(Group_Application_User_USB_DEVICE_Target PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Application_User_USB_DEVICE_Target PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Application_User_USB_DEVICE_Target_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Application_User_USB_DEVICE_Target_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Application_User_USB_DEVICE_Target PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Application_User_USB_DEVICE_Target PUBLIC
  Group_Application_User_USB_DEVICE_Target_ABSTRACTIONS
)

# group Drivers/STM32H7xx_HAL_Driver
add_library(Group_Drivers_STM32H7xx_HAL_Driver OBJECT
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c"
  "${SOLUTION_ROOT}/../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c"
)
target_include_directories(Group_Drivers_STM32H7xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Drivers_STM32H7xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Drivers_STM32H7xx_HAL_Driver_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Drivers_STM32H7xx_HAL_Driver_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Drivers_STM32H7xx_HAL_Driver PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Drivers_STM32H7xx_HAL_Driver PUBLIC
  Group_Drivers_STM32H7xx_HAL_Driver_ABSTRACTIONS
)

# group Drivers/CMSIS
add_library(Group_Drivers_CMSIS OBJECT
  "${SOLUTION_ROOT}/../Core/Src/system_stm32h7xx.c"
)
target_include_directories(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Drivers_CMSIS_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Drivers_CMSIS_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Drivers_CMSIS PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Drivers_CMSIS PUBLIC
  Group_Drivers_CMSIS_ABSTRACTIONS
)

# group Middlewares/USB_Device_Library
add_library(Group_Middlewares_USB_Device_Library OBJECT
  "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c"
  "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c"
  "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c"
  "${SOLUTION_ROOT}/../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c"
)
target_include_directories(Group_Middlewares_USB_Device_Library PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
)
target_compile_definitions(Group_Middlewares_USB_Device_Library PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
add_library(Group_Middlewares_USB_Device_Library_ABSTRACTIONS INTERFACE)
target_link_libraries(Group_Middlewares_USB_Device_Library_ABSTRACTIONS INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)
target_compile_options(Group_Middlewares_USB_Device_Library PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Group_Middlewares_USB_Device_Library PUBLIC
  Group_Middlewares_USB_Device_Library_ABSTRACTIONS
)
