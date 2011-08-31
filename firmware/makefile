#directory definitions
ROOT = ../../..
SCANDAL = $(ROOT)/scandal
ARCH = $(SCANDAL)/src/arch/lpc11c14
UTILS = $(ROOT)/utils
LINKER = $(ARCH)/linker
CMSIS = $(ARCH)/cmsis
DRIVERS = $(ARCH)/driver
STARTUP = $(ARCH)/startup
BUILD=./build# never . or clean will delete everything
SRC = ./src

#define utilities
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy
START_PROGRAMMER = dfu-util
FLASH_CHIP = crt_emu_lpc11_13_nxp
CHECKSUM = checksum

#architecture
MCU = cortex-m0
UC = LPC11C14/301

#define files
PROJECT = main

#LINKSCRIPT = lpc1114_flash_crp.ld
LINKSCRIPT=lpc1114_flash_can_crp.ld
#Should LINKSCRIPT=lpc1114_flash_can_crp.ld, It is slightly different in terms of memory management
#Changed on 23 Aug - Charith, saw something in the user manual to do with some black of memory at the
#top being required by the CAN controller
STARTUPSCRIPT = cr_startup_lpc11

OBJECTS  = $(PROJECT).o scandal_obligations.o
OBJECTS += core_cm0.o system_LPC11xx.o leds.o
OBJECTS += gpio.o timer32.o uart.o can.o i2c.o flash.o
OBJECTS += engine.o error.o manager.o maths.o message.o utils.o
OBJECTS += $(STARTUPSCRIPT).o

#compiler flags
OPTIMISATION = s # we need this to fit our code in the flash at the moment :-(
DEBUG = 

CFLAGS  = -mcpu=$(MCU) -DLPC11C14 -D__NEWLIB__ -D__USE_CMSIS -D__CODE_RED -mthumb -O$(OPTIMISATION) $(DEBUG)
CFLAGS += -I$(SCANDAL)/include # for scandal includes
CFLAGS += -I$(ARCH)/include # for arch drivers
CFLAGS += -I$(ARCH) # for cmsis
CFLAGS += -I./include # for project includes
CFLAGS += -lm #Added lm for math.h

LSCRIPT = $(LINKER)/$(LINKSCRIPT)
LFLAGS  = -nostdlib  -Xlinker -Map=$(BUILD)/$(PROJECT).map -Xlinker --gc-sections -L $(LINKER) -T "$(LSCRIPT)"

.PHONY: all clean program

all: $(PROJECT).axf $(PROJECT).bin

$(PROJECT).axf: $(OBJECTS)
	@$(CC) $(CFLAGS) $(LFLAGS) $(addprefix $(BUILD)/,$(OBJECTS)) -o $(BUILD)/$(PROJECT).axf 

###compile object files individually
$(PROJECT).o: src/$(PROJECT).c
	@mkdir -p $(BUILD)
	@if [ "$(BUILD)" == "." ]; then echo -e "\033[91mDon't use . as BUILD\033[0m"; exit 1; fi
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

scandal_obligations.o: src/scandal_obligations.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

leds.o: src/leds.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

core_cm0.o: $(CMSIS)/core_cm0.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

engine.o: $(SCANDAL)/src/engine.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

error.o: $(SCANDAL)/src/error.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

manager.o: $(SCANDAL)/src/manager.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

message.o: $(SCANDAL)/src/message.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

utils.o: $(SCANDAL)/src/utils.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

maths.o: $(SCANDAL)/src/maths.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

system_LPC11xx.o: $(CMSIS)/system_LPC11xx.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

gpio.o: $(DRIVERS)/gpio.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

timer32.o: $(DRIVERS)/timer32.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

uart.o: $(DRIVERS)/uart.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

can.o: $(DRIVERS)/can.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

flash.o: $(DRIVERS)/flash.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

i2c.o: $(DRIVERS)/i2c.c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

$(STARTUPSCRIPT).o: $(STARTUP)/$(STARTUPSCRIPT).c
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

program: $(PROJECT).axf
	@# see http://support.code-red-tech.com/CodeRedWiki/CommandLineFlashProgramming
	#-$(START_PROGRAMMER) -d 0x471:0xdf55 -c 0 -t 2048 -R -D $(UTILS)/LPCXpressoWIN.enc
	#@sleep 1
	@#program using .axf
	$(FLASH_CHIP) -wire=winusb -p$(UC) -flash-load-exec=$(BUILD)/$(PROJECT).axf
	@#program using .bin, doesn't seem to work atm
	@#sudo $(FLASH_CHIP) -wire=winusb -p$(UC) -flash-load=$(BUILD)/$(PROJECT).bin -load-base=0x0000

clean:
	@if [ "$(BUILD)" == "." ]; then echo -e "\033[91mDon't use . as BUILD\033[0m"; exit 1; fi
	@rm -Rf $(BUILD)

$(PROJECT).bin: $(PROJECT).axf
	@$(SIZE) $(BUILD)/$(PROJECT).axf; 
	$(OBJCOPY) -O binary $(BUILD)/$(PROJECT).axf $(BUILD)/$(PROJECT).bin;
	$(CHECKSUM) -p $(UC) -d $(BUILD)/$(PROJECT).bin;
	@echo
