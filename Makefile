# Location of top-level MicroPython directory
MPY_DIR = micropython

# Name of module (different to built-in uzlib so it can coexist)
MOD = i2cperipheral

# Source files (.c, .S, or .py)
SRC =  CI2CPeripheral.c
SRC += I2CPeripheral.py
SRC += divider.S
SRC += i2c.c
SRC += gpio.c

# Architecture to build for (x86, x64, armv7m, xtensa, xtensawin)
ARCH = armv6m

PICO_SDK_DIR = $(MPY_DIR)/lib/pico-sdk
# Define to optimize out code that we don't need since .mpy linker doesn't do it.
CFLAGS += -DI2CP_OPTIMIZE_SPACE
# Added to get Pico SDK I2C driver to build.
CFLAGS += -I$(PICO_SDK_DIR)/src/rp2_common/hardware_i2c/include
CFLAGS += -I$(PICO_SDK_DIR)/src/common/pico_time/include
CFLAGS += -I$(PICO_SDK_DIR)/src/rp2_common/hardware_timer/include
CFLAGS += -I$(PICO_SDK_DIR)/src/rp2040/hardware_structs/include
CFLAGS += -I$(PICO_SDK_DIR)/src/rp2_common/hardware_base/include
CFLAGS += -I$(PICO_SDK_DIR)/src/rp2_common/hardware_resets/include
CFLAGS += -I$(PICO_SDK_DIR)/src/rp2_common/hardware_clocks/include
# Added to get Pico SDK GPIO driver to build.
CFLAGS += -I$(PICO_SDK_DIR)/src/rp2_common/hardware_gpio/include
CFLAGS += -I$(PICO_SDK_DIR)/src/rp2_common/hardware_sync/include
CFLAGS += -I$(PICO_SDK_DIR)/src/rp2_common/hardware_irq/include
# I added folowing ones for divider driver.
CFLAGS += -I$(PICO_SDK_DIR)/src/rp2040/hardware_regs/include
CFLAGS += -I$(PICO_SDK_DIR)/src/rp2_common/hardware_divider/include
CFLAGS += -I$(PICO_SDK_DIR)/src/rp2_common/pico_platform/include
CFLAGS += -I$(PICO_SDK_DIR)/src/common/pico_base/include

#SRC += $(addprefix $(realpath $(PICO_SDK_DIR))/,\
#	src/rp2_common/hardware_i2c/i2c.c \
#	src/rp2_common/hardware_gpio/gpio.c \
#	)

# The dynruntime.mk won't be properly configured until after init rule runs.
ifneq "$(findstring init,$(MAKECMDGOALS))" "init"
	include $(MPY_DIR)/py/dynruntime.mk
endif

# Pico SDK requires this.
CFLAGS += -std=gnu11



# Extra rules that I have added to setup the environment for building this code.
.PHONY: init diff

init :
	@echo Initializing git submodules...
	@git submodule update --init micropython
	@cd micropython; git submodule update --init lib/pico-sdk
	@echo Patching micropython repository sources for ARMv6m support...
	-@patch -N -p1 <micropython.patch 1>/dev/null ;exit 0
	@echo Building mpy-cross...
	@$(MAKE) --no-print-directory -C micropython/mpy-cross
	@echo Applying diffs to Pico SDK sources...
	@cp $(PICO_SDK_DIR)/src/rp2_common/pico_divider/divider.S divider.S
	@cp $(PICO_SDK_DIR)/src/rp2_common/hardware_i2c/i2c.c i2c.c
	@cp $(PICO_SDK_DIR)/src/rp2_common/hardware_gpio/gpio.c gpio.c
	@patch divider.S divider.S.diff
	@patch i2c.c i2c.c.diff
	@patch gpio.c gpio.c.diff
	@echo Make sure that you have the necessary Python module installed: pip install 'pyelftools>=0.25'

all : diff

diff : divider.S.diff i2c.c.diff gpio.c.diff

divider.S.diff : divider.S
	@echo Calculating diff for $<...
	@diff $(PICO_SDK_DIR)/src/rp2_common/pico_divider/divider.S $< >$@; exit 0

i2c.c.diff : i2c.c
	@echo Calculating diff for $<...
	@diff $(PICO_SDK_DIR)/src/rp2_common/hardware_i2c/i2c.c $< >$@; exit 0

gpio.c.diff : gpio.c
	@echo Calculating diff for $<...
	@diff $(PICO_SDK_DIR)/src/rp2_common/hardware_gpio/gpio.c $< >$@; exit 0
