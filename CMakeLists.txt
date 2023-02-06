cmake_minimum_required(VERSION 3.16)

project(telega_light_control C ASM)

find_package(CMSIS COMPONENTS STM32F1 REQUIRED)
find_package(HAL COMPONENTS STM32F1 REQUIRED)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)



file(GLOB_RECURSE p_headers "include/*.h")

set(PROJECT_SOURCES_WS
	src/can.c
	src/leds.c
	src/main.c
	src/stm32f1xx_hal_msp.c
	src/stm32f1xx_it.c
	src/ws2815.c
	${p_headers}
)

set(PROJECT_SOURCES_PT
	src/can.c
	src/leds_pt.c
	src/main.c
	src/stm32f1xx_hal_msp.c
	src/stm32f1xx_it.c
	${p_headers}
)

INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(EXTRA_FLAGS
#	-Werror
	-fno-common
	-Wall -Wextra
	-Wconversion -Wsign-conversion
	-Wlogical-op -Wcast-align
	-Wclobbered -Wcast-qual -Wchar-subscripts -Wcomment
	-Wdisabled-optimization -Wempty-body -Wfloat-equal -Wformat
	-Wformat-security -Wformat-y2k -Wignored-qualifiers -Wimport
	-Winit-self -Winvalid-pch -Wmissing-braces -Wmissing-declarations
	-Wmissing-field-initializers -Wmissing-noreturn -Wno-attributes
	-Wno-inline -Wno-long-long -Wno-redundant-decls -Wno-variadic-macros
	-Wparentheses -Wpointer-arith -Wreturn-type -Wsequence-point -Wshadow
	-Wsign-compare -Wswitch -Wswitch-default -Wswitch-enum -Wtrigraphs
	-Wtype-limits -Wuninitialized -Wunknown-pragmas
	-Wunreachable-code -Wunused -Wunused-but-set-variable
	-Wunused-function -Wunused-label -Wunused-parameter
	-Wunused-value -Wunused-variable -Wvolatile-register-var
	-Wwrite-strings -Wno-missing-field-initializers
	-D__SOFTFP__
	)
foreach(_w ${EXTRA_FLAGS})
	set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} ${_w}")
endforeach()

set_source_files_properties(
	${PROJECT_SOURCES_WS}
	PROPERTIES
		COMPILE_FLAGS "${EXTRA_C_FLAGS}")

set_source_files_properties(
	${PROJECT_SOURCES_PT}
	PROPERTIES
		COMPILE_FLAGS "${EXTRA_C_FLAGS}")

add_executable(${CMAKE_PROJECT_NAME}_WS ${PROJECT_SOURCES_WS})

add_executable(${CMAKE_PROJECT_NAME}_PT ${PROJECT_SOURCES_PT})

target_compile_definitions(${CMAKE_PROJECT_NAME}_WS PRIVATE CAN_ADDRESS=100)
target_compile_definitions(${CMAKE_PROJECT_NAME}_PT PRIVATE CAN_ADDRESS=101)

target_link_libraries(${CMAKE_PROJECT_NAME}_WS
	CMSIS::STM32::F103C6
	HAL::STM32::F1::UART
	HAL::STM32::F1::CAN
	HAL::STM32::F1::DMA
	HAL::STM32::F1::GPIO
	HAL::STM32::F1::RCC
	HAL::STM32::F1::TIM
	HAL::STM32::F1::TIMEx
	HAL::STM32::F1::CORTEX
	STM32::NoSys
	)

target_link_libraries(${CMAKE_PROJECT_NAME}_PT
	CMSIS::STM32::F103C6
	HAL::STM32::F1::UART
	HAL::STM32::F1::CAN
	HAL::STM32::F1::DMA
	HAL::STM32::F1::GPIO
	HAL::STM32::F1::RCC
	HAL::STM32::F1::TIM
	HAL::STM32::F1::TIMEx
	HAL::STM32::F1::CORTEX
	STM32::NoSys
	)

stm32_generate_binary_file(${CMAKE_PROJECT_NAME}_WS)
stm32_print_size_of_target(${CMAKE_PROJECT_NAME}_WS)

stm32_generate_binary_file(${CMAKE_PROJECT_NAME}_PT)
stm32_print_size_of_target(${CMAKE_PROJECT_NAME}_PT)
