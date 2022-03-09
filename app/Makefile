# The firmware uses the Kbuild build system. There are 'Kbuild' files in this
# example that outlays what needs to be built. (check src/Kbuild).
#
# The firmware is configured using options in Kconfig files, the
# values of these end up in the .config file in the firmware directory.
#
# By setting the OOT_CONFIG (it is '$(PWD)/oot-config' by default) environment
# variable you can provide a custom configuration. It is important that you
# enable the app-layer. See app-config in this directory for example.

#
# We want to execute the main Makefile for the firmware project,
# it will handle the build for us.
#
CRAZYFLIE_BASE := ..

#
# We tell the firmware that this is an "Out-of-Tree" build so look for
# kbuild files in this directory.
#
OOT ?= $(PWD)

#
# We need to tell the firmware where to find extra config options for this
# build.
#
OOT_CONFIG := $(OOT)/app-config

#
# Indicate that the out-of-tree build should use a C++ linker
#
OOT_USES_CXX := 1

OOT_ARGS ?= -C $(CRAZYFLIE_BASE) OOT=$(OOT) OOT_USES_CXX=$(OOT_USES_CXX) EXTRA_CFLAGS=$(EXTRA_CFLAGS)

.PHONY: all clean

all:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) KCONFIG_ALLCONFIG=$(OOT_CONFIG) alldefconfig
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) -j 12

clean:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) clean
	rm -f $(OOT)/src/*.o $(OOT)/src/.*.cmd \
		$(OOT)/vendor/libskybrush/src/lights/*.o $(OOT)/vendor/libskybrush/src/lights/.*.cmd \
		$(OOT)/vendor/*.o $(OOT)/vendor/.*.cmd \
		$(OOT)/*.o $(OOT)/.*.cmd

menuconfig:
	$(MAKE) KBUILD_OUTPUT=$(OOT)/build $(OOT_ARGS) menuconfig

