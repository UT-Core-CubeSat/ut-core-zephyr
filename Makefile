# ============================================ #
#                  IMPORTANT                   #
# ============================================ #
#                                              #
# Keep in mind any changes made to this file   #
# MUST work for both Windows CMD and Bash!     #
#                                              #


ZEPHYR_VERSION := 1.0.1

# APPS := ADCS CDH EPS GNSS MOTOR SOLAR

export ZEPHYR_BASE            := $(CURDIR)/zephyr
export ZEPHYR_SDK_INSTALL_DIR := $(CURDIR)/zephyr/zephyr-sdk-1.0.1
export BOARD_ROOT             := $(CURDIR)
export ROOT                   := $(CURDIR)


WEST_BFLAGS :=
# CMAKE_BFLAGS := -DCONFIG_LOG_DEFAULT_LEVEL=3
CMAKE_BFLAGS := -DCMAKE_BUILD_PARALLEL_LEVEL=8


# Platform specific variables
ifeq ($(OS),Windows_NT)
SYS_PYTHON	:= python
PYBIN		:= .venv/Scripts
RM			:= rmdir /S /Q
OPENOCD		:= $(ZEPHYR_SDK_INSTALL_DIR)/hosttools/openocd/bin/openocd.exe
MKDIR		:= mkdir
else
SYS_PYTHON	:= python3
PYBIN		:= .venv/bin
RM			:= rm -rf
OPENOCD		:= $(ZEPHYR_SDK_INSTALL_DIR)/hosttools/sysroots/x86_64-pokysdk-linux/usr/bin/openocd
MKDIR		:= mkdir -p
endif


GNU		:= $(ZEPHYR_SDK_INSTALL_DIR)/gnu/arm-zephyr-eabi

PYTHON 	:= $(PYBIN)/python
PIP    	:= $(PYBIN)/pip
WEST	:= $(PYBIN)/west
GDB		:= $(GNU)/bin/arm-zephyr-eabi-gdb


# Require APP variable be defined to run build
ifeq ($(MAKECMDGOALS),build)
ifndef APP
$(error APP is not defined)
else
BUILD := build/$(APP)
endif
endif


# Require BUILD variable be defined to run following rules
ifneq ($(filter flash simulate gdb,$(MAKECMDGOALS)),)
ifndef BUILD
$(error BUILD is not defined)
endif
endif


# LOG UART override option
ifdef LOG_UART
CMAKE_BFLAGS += -DCONFIG_PRINTK=y \
				-DCONFIG_LOG=y \
				-DCONFIG_LOG_MODE_IMMEDIATE=y \
				-DCONFIG_SERIAL=y \
				-DCONFIG_LOG_BACKEND_UART=y \
				-DCONFIG_UART_CONSOLE=y \
				-DCONFIG_LOG_BACKEND_RTT=n \
				-DCONFIG_USE_SEGGER_RTT=n \
				-DCONFIG_RTT_CONSOLE=n
endif


# LOG RTT override option
ifdef LOG_RTT
CMAKE_BFLAGS += -DCONFIG_PRINTK=y \
				-DCONFIG_LOG=y \
				-DCONFIG_LOG_MODE_IMMEDIATE=y \
				-DCONFIG_SERIAL=n \
				-DCONFIG_LOG_BACKEND_UART=n \
				-DCONFIG_UART_CONSOLE=n \
				-DCONFIG_LOG_BACKEND_RTT=y \
				-DCONFIG_USE_SEGGER_RTT=y \
				-DCONFIG_RTT_CONSOLE=y
endif


ifdef BOARD
WEST_BFLAGS += -b $(BOARD)
endif


# To quell weird sim errors
export TERM := vt100




help: all


all:
	@echo make ....................... print this help message
	@echo make help .................. print this help message
	@echo make setup ................. setup build environment
	@echo make update ................ update zephyr after change to west config
	@echo make build APP=path ........ build app located at path
	@echo make build-full ............ build all core apps
	@echo make flash BUILD=path ...... flash app binary to board
	@echo make simulate BUILD=path ... launch simulation
	@echo make simulate-full ......... simulate all core apps INCOMPLETE
	@echo make debug ................. launch OpenOCD server to connect to board
	@echo make gdb BUILD=path ........ launch GDB client to debug board
	@echo make listen ................ launch TCP client to view debug logs
	@echo make readme ................ HTTP server to preview GitHub README.md
	@echo make docs .................. build code docs
	@echo make host .................. host HTTP server to view docs
	@echo make clean ................. remove build files and binaries
	@echo make clean-purge ........... remove builds and zephyr environment



update:
	$(WEST) update


setup:
	@: # setup virtual environment
	$(SYS_PYTHON) -m venv .venv
	@: # ensure pip is in the venv
	$(PYTHON) -m pip install --upgrade pip
	@: # install west
	$(PIP) install west
	@: # download zephyr
	$(WEST) update
	@: # install west dependancies in the virtual en
	$(WEST) packages pip --install
	@: # install the zephyr sdk
	cd $(ZEPHYR_BASE)
	$(WEST) sdk install -t arm-zephyr-eabi -d $(ZEPHYR_SDK_INSTALL_DIR) --version $(ZEPHYR_VERSION)
	cd $(CURDIR)

build:
	$(WEST) build $(WEST_BFLAGS) -p auto -d $(BUILD) $(APP) -- $(CMAKE_BFLAGS) -DEXTRA_CFLAGS="$(EXTRA_CFLAGS)"


build-full:
	$(MAKE) build APP=apps/ADCS
	$(MAKE) build APP=apps/CDH
	# $(MAKE) build APP=apps/COMMS
	$(MAKE) build APP=apps/EPS
	$(MAKE) build APP=apps/GNSS
	$(MAKE) build APP=apps/MOTOR
	$(MAKE) build APP=apps/SOLAR


# build-full: $(APPS)
# 
# $(APPS):
# 	$(MAKE) build APP=apps/EPS


build-can-tests:
	$(MAKE) build BUILD=build/apps/test/CANtest1 APP=apps/tests/CANtest EXTRA_CFLAGS="-DNODE_ID=1"
	$(MAKE) build BUILD=build/apps/test/CANtest2 APP=apps/tests/CANtest EXTRA_CFLAGS="-DNODE_ID=2"
	$(MAKE) build BUILD=build/apps/test/CANloop APP=apps/tests/CANtest EXTRA_CFLAGS="-DLOOPBACK_TEST"


# App must be built with UART as a logging backend. e.g. building with `LOG_UART=1`
simulate:
	renode -e 'include @sim/general.resc; sysbus LoadELF @$(BUILD)/zephyr/zephyr.elf;'


# simulate-full:
# 	renode -e 'include @sim/integrated_sim.resc;'


gdb:
	$(GDB) $(BUILD)/zephyr/zephyr.elf -ex "target remote :3333"


debug:
	$(OPENOCD) -f tools/openocd_semihost.cfg

listen:
	telnet localhost 9090

flash:
	$(WEST) flash -d $(BUILD)




clean:
	$(RM) build

clean-purge:
	-$(RM) build
	-$(RM) zephyr
	-$(RM) modules
	-$(RM) bootloader
	-$(RM) .venv

readme:
	gh markdown-preview --disable-reload

docs:
	-@$(MKDIR) build
	doxygen
	@echo Docs successfully built

host:
	$(SYS_PYTHON) -m http.server 8080 --directory build/docs/html

.PHONY: help all setup update
# .PHONY: build build-full build-can-tests flash $(APPS)
.PHONY: build build-full build-can-tests flash $(APPS)
.PHONY: debug simulate simulate-full gdb listen
.PHONY: clean clean-purge readme host docs

