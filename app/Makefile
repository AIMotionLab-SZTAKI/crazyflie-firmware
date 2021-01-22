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

## Set the positioning system in TDoA3 mode on startup
LPS_TDOA3_ENABLE=1

## Weight of the Crazyflie + UWB + LED deck
CFLAGS += -DCF_MASS=0.035f

## Preflight test configuration ------------------------------------

# Specify minimum number of UWB anchors that have to be configured
CFLAGS += -DPREFLIGHT_MIN_LOCO_ANCHOR_COUNT=6

# Specify minimum number of UWB anchors that have to be active
CFLAGS += -DPREFLIGHT_MIN_ACTIVE_LOCO_ANCHOR_COUNT=6

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
PROJ_OBJ += app.o crtp_drone_show_service.o drone_show.o light_program.o preflight.o

# LED light control library
INCLUDES += -Ivendor/ledctrl/include
PROJ_OBJ += libledctrl.a

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


