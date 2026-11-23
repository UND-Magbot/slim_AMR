# UND_ARM - STM32H743VITx  (CubeIDE -> Linux Makefile 이식판)
# 빌드: make -j$(nproc)   /  플래싱: make flash  /  정리: make clean

TARGET = UND_ARM
DEBUG  = 1
OPT    = -O0
BUILD_DIR = build

######################################
# 소스
######################################
C_SOURCES =  \
Core/Src/can.c \
Core/Src/can2.c \
Core/Src/gpio.c \
Core/Src/l2db4830.c \
Core/Src/lift.c \
Core/Src/main.c \
Core/Src/rs485.c \
Core/Src/RS485_KincoFD125.c \
Core/Src/stm32h7xx_hal_msp.c \
Core/Src/stm32h7xx_it.c \
Core/Src/syscalls.c \
Core/Src/sysmem.c \
Core/Src/system.c \
Core/Src/system_stm32h7xx.c \
Core/Src/tim.c \
Core/Src/tricycle.c \
Core/Src/usart.c \
Core/Src/usbcdc.c \
Core/ThreadSafe/newlib_lock_glue.c \
USB_DEVICE/App/usbd_cdc_if.c \
USB_DEVICE/App/usbd_desc.c \
USB_DEVICE/App/usb_device.c \
USB_DEVICE/Target/usbd_conf.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c \
Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c \
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c

ASM_SOURCES = Core/Startup/startup_stm32h743vitx.s

######################################
# 툴체인
######################################
PREFIX = arm-none-eabi-
ifdef GCC_PATH
CC  = $(GCC_PATH)/$(PREFIX)gcc
AS  = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP  = $(GCC_PATH)/$(PREFIX)objcopy
SZ  = $(GCC_PATH)/$(PREFIX)size
OD  = $(GCC_PATH)/$(PREFIX)objdump
else
CC  = $(PREFIX)gcc
AS  = $(PREFIX)gcc -x assembler-with-cpp
CP  = $(PREFIX)objcopy
SZ  = $(PREFIX)size
OD  = $(PREFIX)objdump
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

######################################
# CPU / FPU  (CubeIDE 설정 그대로)
######################################
CPU    = -mcpu=cortex-m7
FPU    = -mfpu=fpv5-d16
FLOAT-ABI = -mfloat-abi=hard
MCU    = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

AS_DEFS =
C_DEFS  =  \
-DUSE_PWR_LDO_SUPPLY \
-DUSE_HAL_DRIVER \
-DSTM32H743xx \
-DSTM32_THREAD_SAFE_STRATEGY=2

AS_INCLUDES =
C_INCLUDES =  \
-ICore/Inc \
-ICore/ThreadSafe \
-IDrivers/STM32H7xx_HAL_Driver/Inc \
-IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
-IDrivers/CMSIS/Include \
-IUSB_DEVICE/App \
-IUSB_DEVICE/Target \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

# 주의: CubeIDE의 -fcyclomatic-complexity 는 ST 전용 GCC 확장이라 제외함
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS  = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -std=gnu11 -Wall -fdata-sections -ffunction-sections -fstack-usage

ifeq ($(DEBUG), 1)
CFLAGS += -g3 -gdwarf-2 -DDEBUG
endif

CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

######################################
# 링크
######################################
LDSCRIPT = STM32H743VITX_FLASH.ld
LIBS = -lc -lm
LIBDIR =
LDFLAGS = $(MCU) --specs=nosys.specs --specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) \
-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -static \
-u _printf_float -u _scanf_float

######################################
# 빌드 규칙
######################################
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
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
	mkdir -p $@

######################################
# 플래싱 / 디버그
######################################
OPENOCD_IF ?= interface/stlink.cfg
OPENOCD_TG ?= target/stm32h7x.cfg

flash: $(BUILD_DIR)/$(TARGET).elf
	openocd -f $(OPENOCD_IF) -f $(OPENOCD_TG) \
	  -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

# st-flash(stlink-tools) 사용 시
flash-st: $(BUILD_DIR)/$(TARGET).bin
	st-flash --reset write $(BUILD_DIR)/$(TARGET).bin 0x8000000

# GDB 서버 (다른 터미널에서 arm-none-eabi-gdb build/UND_ARM.elf -ex "target extended-remote :3333")
gdbserver:
	openocd -f $(OPENOCD_IF) -f $(OPENOCD_TG)

# ST-LINK 없이 플래싱 --------------------------------------------------------
# DFU: BOOT0=1 로 놓고 리셋 -> USB SLAVE 포트가 0483:df11 로 열거됨
# 주의: ":leave" 는 붙이지 않는다. 부트로더가 앱으로 점프하면서 마지막
#       GET_STATUS 에 응답하지 않아 dfu-util 0.9 가 쓰기 성공 후에도 에러로 종료한다.
#       어차피 BOOT0 을 0 으로 되돌리고 리셋해야 하므로 필요 없다.
dfu: $(BUILD_DIR)/$(TARGET).bin
	dfu-util -a 0 -s 0x08000000 -D $(BUILD_DIR)/$(TARGET).bin

# UART 부트로더: BOOT0=1 로 놓고 리셋 -> USART1(PA9/PA10, 온보드 CH340)
SERIAL_PORT ?= /dev/ttyUSB0
flash-uart: $(BUILD_DIR)/$(TARGET).bin
	stm32flash -w $(BUILD_DIR)/$(TARGET).bin -v -g 0x08000000 -b 115200 $(SERIAL_PORT)

size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) $<

clean:
	-rm -rf $(BUILD_DIR)

.PHONY: all clean flash flash-st dfu flash-uart gdbserver size

-include $(wildcard $(BUILD_DIR)/*.d)
