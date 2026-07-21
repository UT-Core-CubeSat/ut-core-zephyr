
ZEPHYR_VERSION := 1.0.1


export ZEPHYR_BASE            := $(CURDIR)/zephyr
export ZEPHYR_SDK_INSTALL_DIR := $(CURDIR)/zephyr/zephyr-sdk-1.0.1
export BOARD_ROOT             := $(CURDIR)
export ROOT                   := $(CURDIR)


WEST_BFLAGS :=
# CMAKE_BFLAGS := -DCONFIG_LOG_DEFAULT_LEVEL=3
CMAKE_BFLAGS :=


PYTHON := .venv/bin/python
PIP    := .venv/bin/pip
WEST   := .venv/bin/west


ifeq ($(OS),Windows_NT)
SYS_PYTHON := python
else
SYS_PYTHON := python3
endif


# Require APP variable be defined to run build
ifeq ($(MAKECMDGOALS),build)
ifndef APP
$(error APP is not defined)
else
BUILD := build/$(APP)
endif
endif


# Require BUILD variable be defined to run following rules
ifneq ($(filter flash debug simulate gdb,$(MAKECMDGOALS)),)
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


help: all


all:
	@echo make setup
	@echo make update
	@echo "make build APP=<APP>"
	@echo make clean
	@echo make clean-purge


update:
	$(WEST) update


setup:
	$(SYS_PYTHON) -m venv .venv				# setup virtual environment
	$(PYTHON) -m pip install --upgrade pip 	# ensure pip is in the venv
	$(PIP) install west						# install west
	$(WEST) update							# download zephyr
	$(WEST) packages pip --install			# install west dependancies in the virtual en
	cd $(ZEPHYR_BASE)						# install the zephyr sdk
	$(WEST) sdk install -t arm-zephyr-eabi -d $(ZEPHYR_SDK_INSTALL_DIR) --version $(ZEPHYR_VERSION)
	cd $(CURDIR)

build:
	$(WEST) build $(WEST_BFLAGS) -p auto -d $(BUILD) $(APP) -- $(CMAKE_BFLAGS) -DEXTRA_CFLAGS="$(EXTRA_CFLAGS)"


build-full:
	$(MAKE) build APP=apps/cdhMain
	$(MAKE) build APP=apps/GNSS
	$(MAKE) build APP=apps/EPS
	$(MAKE) build APP=apps/solarconglomMain

build-can-tests:
	$(MAKE) build BUILD=build/apps/test/CANtest1 APP=apps/test/CANtest EXTRA_CFLAGS="-DNODE_ID=1"
	$(MAKE) build BUILD=build/apps/test/CANtest2 APP=apps/test/CANtest EXTRA_CFLAGS="-DNODE_ID=2"
	$(MAKE) build BUILD=build/apps/test/CANloop APP=apps/test/CANtest EXTRA_CFLAGS="-DLOOPBACK_TEST"


simulate:
	renode -e 'include @boards/arm/ut_core/support/ut_core.resc; sysbus LoadELF @$(BUILD)/zephyr/zephyr.elf;'


# simulate-full:
# 	renode -e 'include @sim/integrated_sim.resc;'


gdb:
	gdb-multiarch $(BUILD)/zephyr/zephyr.elf -ex "target remote :3333"


debug:
	openocd -f tools/openocd_semihost.cfg

listen:
	telnet localhost 9090

flash:
	$(WEST) flash -d $(BUILD)




clean:
	rm -rf build


clean-purge:
	rm -rf build
	rm -rf zephyr
	rm -rf modules
	rm -rf bootloader
	rm -rf .venv

readme:
	gh markdown-preview --disable-reload

.PHONY: setup update
.PHONY: build build-all build-can-tests flash
.PHONY: debug simulate simulate-full gdb listen
.PHONY: clean clean-purge readme
