ifeq ($(OS),Windows_NT)
SYS_PYTHON := python
else
SYS_PYTHON := python3
endif

GITROOT := $(shell git rev-parse --show-toplevel)
APPPATH := $(shell git rev-parse --show-prefix)

# Air bearing demo build. Selects src/mainAirBearing.cpp and the airBearingDemo
# ADCSCore instead of the orbit flight software. Uses its own build directory so
# the two firmwares keep separate CMake caches and can coexist.
#
# CMAKE_BFLAGS is restated in full rather than appended to, because a variable
# passed on the command line to the root Makefile replaces its value outright
# instead of adding to it. Consequence: AIR_BEARING does not combine with
# LOG_UART/LOG_RTT. Keep the baseline flag below in sync with the root Makefile.
AIR_BUILD  := build/$(APPPATH:%/=%)_airbearing
AIR_BFLAGS := -DCMAKE_BUILD_PARALLEL_LEVEL=8 -DAIR_BEARING=ON

all: build

build:
	$(MAKE) -C $(GITROOT) build APP=$(APPPATH)

build-air:
	$(MAKE) -C $(GITROOT) build APP=$(APPPATH) BUILD=$(AIR_BUILD) CMAKE_BFLAGS="$(AIR_BFLAGS)"

flash-air:
	$(MAKE) -C $(GITROOT) flash BUILD=$(AIR_BUILD)

build-sim:
	$(MAKE) -C $(GITROOT) build APP=$(APPPATH) LOG_UART=1

sim:
	$(MAKE) -C $(GITROOT) simulate BUILD=build/$(APPPATH)

gdb:
	$(MAKE) -C $(GITROOT) gdb BUILD=build/$(APPPATH)

flash:
	$(MAKE) -C $(GITROOT) flash BUILD=build/$(APPPATH)

debug:
	$(MAKE) -C $(GITROOT) debug BUILD=build/$(APPPATH)

listen:
	$(MAKE) -C $(GITROOT) listen

clean:
	$(MAKE) -C $(GITROOT) clean

help:
	@echo make help ........ print this help message
	@echo make build ....... build source for hardware
	@echo make build-air ... build air bearing demo instead of orbit flight SW
	@echo make build-sim ... build source for simulation
	@echo make flash ....... flashes binary to board
	@echo make flash-air ... flashes air bearing demo binary to board
	@echo make sim ......... launches simulation
	@echo make debug ....... launches OpenOCD server to connect to board
	@echo make gdb ......... launches GDB client to debug board
	@echo make listen ...... launches TCP client to view debug logs
	@echo make clean ....... removes build files and binaries

.PHONY: build build-air build-sim debug flash flash-air sim gdb listen clean help
