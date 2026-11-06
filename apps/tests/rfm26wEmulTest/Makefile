# Host tests: run on this computer (native_sim), not on the board. Linux or WSL only.
GITROOT := $(shell git rev-parse --show-toplevel)
APPPATH := $(shell git rev-parse --show-prefix)

all: build

build:
	$(MAKE) -C $(GITROOT) build APP=$(APPPATH)

test:
	$(MAKE) -C $(GITROOT) test APP=$(APPPATH)

clean:
	$(MAKE) -C $(GITROOT) clean

help:
	@echo make help ........ print this help message
	@echo make build ....... build the test for the host
	@echo make test ........ build and run the tests
	@echo make clean ....... removes build files and binaries

.PHONY: all build test clean help
