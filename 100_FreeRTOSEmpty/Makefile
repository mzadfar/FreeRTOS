# Generic Makefile (based on gcc)

#Path to stlink folder for uploading code to board                         
#It does not needed if st-flash --version is recognized in terminal
#which means st-flash command can be used globally
STLINK = ~/Documents/Workspace/STM32/stlink-testing/bin

#Target
TARGET = final

#Building variables
# debug build?
DEBUG = 1

#Semihosting variables
# debug semihosting using OpenOCD
SEMI_HOSTING = 0

# optimization
OPT = -O0 # original: -Og

#Build path
BUILD_DIR = build

# C sources
C_SOURCES =  \
Core/src/main.c \
Core/src/system_stm32f4xx.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/misc.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_can.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_crc.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_aes.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_des.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_tdes.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fsmc.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_md5.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_sha1.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c \
Third-Party/FreeRTOS/org/Source/croutine.c \
Third-Party/FreeRTOS/org/Source/event_groups.c \
Third-Party/FreeRTOS/org/Source/list.c \
Third-Party/FreeRTOS/org/Source/queue.c \
Third-Party/FreeRTOS/org/Source/stream_buffer.c \
Third-Party/FreeRTOS/org/Source/tasks.c \
Third-Party/FreeRTOS/org/Source/timers.c \
Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F/port.c \
Third-Party/FreeRTOS/org/Source/portable/MemMang/heap_4.c 

ifeq ($(SEMI_HOSTING), 0)
C_SOURCES +=  \
Core/src/syscalls.c 
endif

# ASM sources
ASM_SOURCES =  \
Startup/startup_stm32f429_439xx.s

#Binaries
PREFIX = arm-none-eabi-

# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

# CFLAGS
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI) -mthumb-interwork -mlittle-endian -std=gnu11

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_STDPERIPH_DRIVER \
-DUSE_HAL_DRIVER \
-DSTM32F439xx

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-ICore/inc \
-ILibraries/STM32F4xx_StdPeriph_Driver/inc \
-ILibraries/CMSIS/Device/ST/STM32F4xx/Include \
-ILibraries/CMSIS/Include \
-IThird-Party/FreeRTOS/org/Source \
-IThird-Party/FreeRTOS/org/Source/include \
-IThird-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F \
-IConfig
#-IUtilities/STM32_EVAL 

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g3 -gdwarf-2 # original: -g
endif

# My C flags:
CFLAGS += -Wno-unused-variable

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# LDFLAGS
# link script
LDSCRIPT = STM32F439NIHx_FLASH.ld

# libraries
ifeq ($(SEMI_HOSTING), 0)
LIBS = -specs=nano.specs -lc -lm -lnosys 
LDFLAGS = $(MCU) $(LIBS) -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
else
LIBS =  -specs=rdimon.specs -lc -lrdimon -lm -lnosys 
LDFLAGS = $(MCU) $(LIBS) -T$(LDSCRIPT)  -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
endif

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@	

# clean up
clean:
	-rm -fR $(BUILD_DIR)
#rm -rf *.o *.elf *.bin *.hex *.map $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin

# dependencies
-include $(wildcard $(BUILD_DIR)/*.d)

# Flash with J-Link
# Configure device name, everything else should remain the same
jflash: $(BUILD_DIR)/jflash
	JLinkExe -commanderscript $<

# Change to yours
device = STM32F439ZI

# First create a file with all commands
$(BUILD_DIR)/jflash: $(BUILD_DIR)/$(TARGET).bin
	@touch $@
	@echo device $(device) > $@
	@echo -e si 1'\n'speed 4000 >> $@
	@echo loadbin $< 0x8000000 >> $@
	@echo -e r'\n'g'\n'qc >> $@

# FLash with ST-LINK
stflash: $(BUILD_DIR)/$(TARGET).bin
	st-flash --reset write $< 0x8000000

# Flash with UART module
# If you have problem with flashing but it does connect,
# remove '-e 0' so that it will erase flash contents and
# flash firmware fresh
uflash: $(BUILD_DIR)/$(TARGET).bin
# This one is used if you have UART module with RTS and DTR pins
	stm32flash -b 115200 -e 0 -R -i rts,dtr,-rts:rts,-dtr,-rts -v -w $< $(PORT)
# Else use this one and manualy set your MCU to bootloader mode
#stm32flash -b 115200 -e 0 -v -w $< $(PORT)

load:
	openocd -f board/st_nucleo_f4.cfg
