##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.6.0] date: [Thu Jan 16 15:09:08 EST 2020]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = MTE_380_Robot


######################################
# building variables
######################################
# debug build?
DEBUG ?= 1
# optimization
ifeq ($(DEBUG), 1)
OPT = -Og
else
OPT = -flto -O2 # change to -O3, -Os, or -Ofast as needed
endif


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
CXX_SOURCES = \
Core/Src/main.cc

# C sources
CORE_SOURCES = \
Core/Src/freertos.c \
Core/Src/gpio.c \
Core/Src/i2c.c \
Core/Src/mpu9250.c \
Core/Src/panic.c \
Core/Src/retarget.c \
Core/Src/startup.c \
Core/Src/stm32f7xx_hal_msp.c \
Core/Src/stm32f7xx_hal_timebase_tim.c \
Core/Src/stm32f7xx_it.c \
Core/Src/system_stm32f7xx.c \
Core/Src/tim.c \
Core/Src/usart.c

HAL_SOURCES = \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_can.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_crc_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_crc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dac_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dac.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_iwdg.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_lptim.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rng.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rtc_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rtc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart_ex.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_usart.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_adc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_crc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_dac.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_dma.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_exti.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_gpio.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_i2c.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_lptim.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_pwr.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_rcc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_rng.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_rtc.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_spi.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_tim.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usart.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_utils.c

FREERTOS_SOURCES = \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c

VLX_SOURCES = \
Middlewares/ST/VLx/core/src/vl53l0x_api_calibration.c \
Middlewares/ST/VLx/core/src/vl53l0x_api_core.c \
Middlewares/ST/VLx/core/src/vl53l0x_api_ranging.c \
Middlewares/ST/VLx/core/src/vl53l0x_api_strings.c \
Middlewares/ST/VLx/core/src/vl53l0x_api.c \
Middlewares/ST/VLx/platform/src/vl53l0x_platform.c \
Middlewares/ST/VLx/platform/src/vl53l0x_platform_log.c \

C_SOURCES = $(CORE_SOURCES) $(HAL_SOURCES) $(FREERTOS_SOURCES) $(VLX_SOURCES)

# ASM sources
ASM_SOURCES =


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
CXX = $(GCC_PATH)/$(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size -A -x
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m7

# fpu
FPU = -mfpu=fpv5-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc

# C defines
C_DEFS =  \
-D__HEAP_SIZE=0x2000 \
-D__STACK_SIZE=0x2000 \
-DUSE_HAL_DRIVER \
-DSTM32F767xx \
-DARM_MATH_CM7


# AS includes
AS_INCLUDES =  \
-ICore/Inc

# C includes
C_INCLUDES =  \
-ICore/Inc \
-IDrivers/STM32F7xx_HAL_Driver/Inc \
-IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Include \
-IDrivers/CMSIS/DSP/Include \
-IDrivers/CMSIS/Device/ST/STM32F7xx/Include \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 \
-IMiddlewares/ST/VLx/core/inc \
-IMiddlewares/ST/VLx/platform/inc


# compile gcc flags
WARN = -Wall -Wextra -Wno-unused-parameter
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) $(WARN)
CFLAGS += -fno-builtin -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf -fverbose-asm -fstack-usage -save-temps=obj
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# C++ specific flags
CXXFLAGS = $(CFLAGS) -fno-rtti -fno-exceptions -fno-unwind-tables -Wno-missing-field-initializers

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F767ZITx_FLASH.ld

# libraries
LIBS = -lm -larm_cortexM7lfdp_math
LIBDIR = -LDrivers/CMSIS/Lib/GCC
LDFLAGS = $(MCU) $(OPT) -specs=nosys.specs -specs=nano.specs -T$(LDSCRIPT) \
$(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
.PHONY: all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	@$(SZ) $<

#######################################
# build the application
#######################################
# list of C++ objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cc=.o)))
vpath %.cc $(sort $(dir $(CXX_SOURCES)))
# list of objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.cc Makefile | $(BUILD_DIR)
	@echo "CXX	$<"
	@$(CXX) -c $(CXXFLAGS) -std=gnu++17 -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cc=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "CC	$<"
	@$(CC) -c $(CFLAGS) -std=gnu11 -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "AS	$<"
	@$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) Makefile
	@echo "LINK	$@"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "OBJCOPY	$@"
	@$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo "OBJCOPY	$@"
	@$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# clean up
#######################################
.PHONY: clean
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
