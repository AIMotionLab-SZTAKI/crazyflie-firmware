## General app configuration ---------------------------------------

# The Crazyflie model that we are building the firmware for. Possible values:
#
# stock: basic Crazyflie 2.1 with LED ring and UWB module
# show-proto-v1: show drone prototype sent to us by Bitcraze in Dec 2020
# show-proto-v2: Nina's show drone prototype with Lighthouse instead of UWB
#
# When adding a new model, do not forget to create the appropriate file in conf/

CF_MODEL=stock

## Variables derived from the general app configuration ------------

include conf/$(CF_MODEL).mk

# Enable app support
APP=1

# This is the stack size of the task that the default Crazyflie app handler
# allocates on its own. We don't use that, so we set it to a bare minimum.
# Our _own_ stack size is defined in another variable below.
APP_STACKSIZE=16

# This is the _real_ stack size of our task.
DRONE_SHOW_TASK_STACKSIZE=300

## Turn off the LED ring by default
CFLAGS += -DLEDRING_DEFAULT_EFFECT=0

## Do not send CRTP messages when parameter values are updated
CFLAGS += -DSILENT_PARAM_UPDATES

## Do not print debug messages from show mode
CFLAGS += -DSHOW_MODE_SILENT

## Set the positioning system in TDoA3 mode on startup. This gives a minimal
# performance gain at startup and also makes it somewhat easier to detect when
# nodes are misconfigured (mixed TDoA3 and TDoA2 mode), but its effect is
# negligible so we keep it off.
# LPS_TDOA3_ENABLE=1

## Weight of the Crazyflie + UWB + LED deck
CFLAGS += -DCF_MASS=$(CF_MODEL_MASS)

## Preflight test configuration ------------------------------------

ifeq ($(CF_MODEL_POSITIONING),uwb)
# Specify minimum number of UWB anchors that have to be configured and that have
# to be active in order for the preflight checks to pass
CFLAGS += -DPREFLIGHT_MIN_LOCO_ANCHOR_COUNT=6 -DPREFLIGHT_MIN_ACTIVE_LOCO_ANCHOR_COUNT=6
endif

# Specify minimum number of trajectories that have to be uploaded before flight
CFLAGS += -DPREFLIGHT_MIN_TRAJECTORIES=1

# Specify minimum number of light programs that have to be uploaded before flight
CFLAGS += -DPREFLIGHT_MIN_LIGHT_PROGRAMS=1

# CLOAD_CMDS = -w radio://0/30/2M
# DEBUG=1

## Main build configuration ----------------------------------------

VPATH += src/
INCLUDES += -Iinterface/

# Drone show module
PROJ_OBJ += app.o arming.o crtp_drone_show_service.o drone_show.o \
            gcs_light_effects.o light_program.o preflight.o

# LED light control library
INCLUDES += -Ivendor/ledctrl/include
PROJ_OBJ += libledctrl.a

# Make sure that the program name includes the Crazyflie model
PROG=cf2-skybrush-$(CF_MODEL)

# Inherit everything else from the base firmware makefile
CRAZYFLIE_BASE=..
include $(CRAZYFLIE_BASE)/Makefile

# Some of the project is C++-based so we need a C++ compiler
CXX = $(CROSS_COMPILE)g++
LD = $(CROSS_COMPILE)g++

# Add new targets here in order not to override the 'all' target and provide
# a new default target accidentally
libledctrl.a:
	+$(MAKE) -C tools/make/ledctrl/ CRAZYFLIE_BASE=$(abspath $(CRAZYFLIE_BASE)) PROJ_ROOT=$(CURDIR) V=$(V) CROSS_COMPILE=$(CROSS_COMPILE)


