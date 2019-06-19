#Select which microcontroller and board are being used
if (NOT TARGET_CONTROLLER)
  set(TARGET_CONTROLLER "STM32F30X" CACHE STRING "Target MMC Controller")
endif()

if (NOT TARGET_BOARD_NAME)
  set(TARGET_BOARD_NAME "eRTM14" CACHE STRING "Board Name")
endif()

#List all modules used by this board
set(TARGET_MODULES
  "UART_DEBUG"
  "PAYLOAD"
  "SDR"
  "SENSOR"
  "TMP100"
  "EEPROM_24AA025E48"
  "FRU"
  "STM32F3_ADC"
  )

set(BOARD_PATH ${CMAKE_CURRENT_SOURCE_DIR})

#Include the modules sources
set(PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/board_led.c)
set(PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/board_ipmb.c)
set(PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/board_uart.c)
set(PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/pin_mapping.c)
set(PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/i2c_mapping.c)
set(PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/sdr_list.c)

if (";${TARGET_MODULES};" MATCHES ";PAYLOAD;")
  set(PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/payload.c)
  set(MODULES_FLAGS "${MODULES_FLAGS} -DMODULE_PAYLOAD")
endif()

if (";${TARGET_MODULES};" MATCHES ";FRU;")
  set(PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/board_fru.c)
  set(MODULES_FLAGS "${MODULES_FLAGS} -DMODULE_FRU")
endif()

if (";${TARGET_MODULES};" MATCHES ";STM32F3_ADC;")
  set(PROJ_SRCS ${PROJ_SRCS} ${BOARD_PATH}/board_stm32f3_adc.c)
  set(MODULES_FLAGS "${MODULES_FLAGS} -DMODULE_STM32F3_ADC")  # needed?
endif()


#Extra definitions
if (FRU_WRITE_EEPROM)
  message(STATUS "FRU EEPROM will be written if no valid data is found!")
  set(MODULES_FLAGS "${MODULES_FLAGS} -DFRU_WRITE_EEPROM")
  set(FRU_WRITE_EEPROM false CACHE BOOL "Set this flag to enable writing the board ID on the EEPROM" FORCE)
endif()

#Set the variables in the main scope
set(TARGET_MODULES ${TARGET_MODULES} PARENT_SCOPE)
set(MODULES_FLAGS ${MODULES_FLAGS} PARENT_SCOPE)
set(PROJ_SRCS ${PROJ_SRCS} PARENT_SCOPE)
set(PROJ_HDRS ${PROJ_HDRS} ${BOARD_PATH})
set(PROJ_HDRS ${PROJ_HDRS} PARENT_SCOPE)
