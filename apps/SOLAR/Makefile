ifeq ($(OS),Windows_NT)
SYS_PYTHON := python
else
SYS_PYTHON := python3
endif

GITROOT := $(shell git rev-parse --show-toplevel)
APPPATH := $(shell git rev-parse --show-prefix)

all: build

build:
	$(MAKE) -C $(GITROOT) build APP=$(APPPATH)

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
	@echo make build-sim ... build source for simulation
	@echo make flash ....... flashes binary to board
	@echo make sim ......... launches simulation
	@echo make debug ....... launches OpenOCD server to connect to board
	@echo make gdb ......... launches GDB client to debug board
	@echo make listen ...... launches TCP client to view debug logs
	@echo make clean ....... removes build files and binaries

.PHONY: build debug flash sim gdb listen clean help
