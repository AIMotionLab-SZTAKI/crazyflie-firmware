include obj.mk

BIN=$(PROJ_ROOT)/bin/vendor
PROJ_BIN=$(PROJ_ROOT)/bin
LIBSKYBRUSH_SRC=$(PROJ_ROOT)/vendor/libskybrush/src
LIBSKYBRUSH_INC=$(PROJ_ROOT)/vendor/libskybrush/include

VPATH += $(BIN)
VPATH += $(LIBSKYBRUSH_SRC)/lights/


LIB=$(PROJ_BIN)/libskybrush.a

CSFLAGS = -O2 -I$(LIBSKYBRUSH_INC) -I. -mfloat-abi=hard -mfpu=fpv4-sp-d16
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

$(LIB): $(LIBSKYBRUSH_OBJ)
	@$(AR) rcs $@ $(foreach o,$(LIBSKYBRUSH_OBJ),$(BIN)/$(o))
	@echo "  AR    libskybrush.a"

include ../targets.mk
