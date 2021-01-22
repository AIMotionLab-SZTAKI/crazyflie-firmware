include obj.mk

BIN=$(PROJ_ROOT)/bin/vendor
PROJ_BIN=$(PROJ_ROOT)/bin
LEDCTRL_SRC=$(PROJ_ROOT)/vendor/ledctrl/src
LEDCTRL_INC=$(PROJ_ROOT)/vendor/ledctrl/include

VPATH += $(BIN)
VPATH += $(LEDCTRL_SRC)/core/


LIB=$(PROJ_BIN)/libledctrl.a

CSFLAGS = -O2 -I$(LEDCTRL_INC) -I. -mfloat-abi=hard -mfpu=fpv4-sp-d16
CSFLAGS += -mcpu=cortex-m4 -mthumb -ffunction-sections -fdata-sections
CFLAGS=$(CSFLAGS) -include stdint.h

CROSS_COMPILE     ?= arm-none-eabi-
AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
LD = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy
GDB = $(CROSS_COMPILE)gdb

all: $(LIB)

$(LIB): $(LEDCTRL_OBJ)
	@$(AR) rcs $@ $(foreach o,$(LEDCTRL_OBJ),$(BIN)/$(o))
	@echo "  AR    libledctrl.a"

include ../targets.mk
