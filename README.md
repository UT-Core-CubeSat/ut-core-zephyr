# UT CubeSat - Zephyr RTOS Development


Custom STM32U5A5RJTXQ6Q board firmware for our CubeSat project using Zephyr RTOS.
Hardware

**MCU:** STM32U5A5RJTXQ6Q

**Board:** Custom UT CubeSat STM32U5 (ut_core)


<!-- For testing the installation instructions on a new debian system -->
<!-- sudo docker run -it --rm -v $(pwd):/mnt debian:latest bash -->


For contributing, see [CONTRIBUTING.md](CONTRIBUTING.md).

For documentation, see [UT-Core Docs](https://ut-core-cubesat.github.io/ut-core-docs/).


## Contents


- [Project Structure](#project-structure)
- [App Structure](#app-structure)
- [How to Build](#how-to-build)
    - [Install dependancies (Windows)](#install-dependancies-windows)
    - [Install dependancies (Debian/Ubuntu)](#install-dependancies-debianubuntu)
    - [Setup environment and install Zephyr)](#setup-environment-and-install-zephyr)
    - [Build App](#build-app)
- [How to Flash](#how-to-flash)
- [How to view board logs](#how-to-view-board-logs)
- [How to run GDB debugger](#how-to-run-gdb-debugger)
- [How to simulate boards](#how-to-simulate-boards)
- [How to build and host docs](#how-to-build-and-host-docs)
- [Resources](#resources)


---

## Project Structure
<span id="project-structure"></span>

```
ut-core-zephyr/             Git root
├── apps/                   Location for Zephyr applications for each board
│   ├── ...
│   └── test/               Location for testing applications
├── boards/
│   └── arm/
│       └── ut_core/        Zephyr board hardware definitions
├── build/                  Generated output where binary is located
├── common/                 Shared code between apps
├── docs/                   Source files for documentation
├── sim/                    Location for simulation scripts
├── tools/                  Misc. tools for testing
├── .west/
└── zephyr/                 Zephyr operating system. Download by running `make setup` or `make update`
```


## App Structure

```
apps/<appname>/             Application directory
├── boards/
│   └── ut_core.overlay     App specific hardware definitions
├── src/                    Source code
│   ├── ...
│   └── main.c
├── CMakeLists.txt          CMake config for build definitions
├── Makefile                Convenience makefile with local rules. Run `make` to build
└── prj.conf                Zephyr config file. Used for enabling/disabling subsystems, etc.
```



---

## How to Build


### Install dependancies (Windows)

First, install these programs (skip if using `choco`):

1. Install [CMake](https://cmake.org/)
2. Install [Python](https://www.python.org/)
3. Install [Devicetree Compiler](https://www.devicetree.org/)

Next, install these dependancies

```cmd
# With winget
winget install Kitware.CMake Ninja-build.Ninja oss-winget.gperf Python.Python.3.12 Git.Git oss-winget.dtc wget 7zip.7zip

# OR with choco
choco install cmake ninja make python3 git wget 7zip gperf dtc-msys2 openocd gcc-arm-embedded -y
```

If any problems arise, consult Zephyr's documentation for [Getting Started](https://docs.zephyrproject.org/latest/develop/getting_started/index.html).


### Install dependancies (Debian/Ubuntu)

```bash
# update apt
sudo apt update

# install Zephyr dependancies
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget python3-dev python3-venv python3-tk \
  xz-utils file make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1

# install dependancies for debugging (optional)
sudo apt install openocd gdb-multiarch
```


### Setup environment and install Zephyr

```bash
make setup
```

### Build board

```bash
# from app directory
cd apps/<appname>
make build

# OR from git root directory
make build APP=apps/<appname>
```

> [!IMPORTANT]
> Keep in mind UART vs RTT as a logging backend! If you don't see logs outputs when listening to your board such as with `make listen`, then you may be using the wrong logging backend. Change the backend in the app's `prj.conf`, or specify `LOG_UART=1` or `LOG_RTT=1` to force the logging backend in the Makefile.


---

## How to Flash

Install the stm32 flasher tool at https://www.st.com/en/development-tools/stm32cubeprog.html

If on Ubuntu/Debian, Give user udev-rules access (if on windows then skip this step)

```bash
sudo apt install stlink-tools
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Connect computer to board to flash via the ST-Link cable, then flash to board.

```bash
# From app directory
cd apps/<appname>
make flash

# OR From git root directory
make flash BUILD=build/<appname>
```


--- 

## How to view board logs

Whilst connected to the board with the ST-Link cable, start a debug server over the link in the first terminal by running `make debug` or `openocd -f tools/openocd_semihost.cfg`. In a second terminal start a telnet client by running `make listen` or `telnet localhost 9090`, which if successful will print out the debug logs.

```bash
# in the first terminal
make debug

# in a second terminal
make listen
```


---

## How to run GDB debugger

Whilst connected to the board with the ST-Link cable, start a debug server over the link in the first terminal by running `make debug` or `openocd -f tools/openocd_semihost.cfg`. In a second terminal start a gdb client with `make gdb BUILD=build/<appname>`.

```bash
# in the first terminal
make debug

# in a second terminal from the app directory
cd app/<appname>
make gdb

# OR from the git root directory
make gdb BUILD=build/<appname>
```


---

## How to simulate boards

> [!WARNING] 
> Full simulation is not implemented!

Only one board at a time can be simulated. As of writing this, most peripherals are not implemented in the simulation. For a complete and reliable test of software, run tests on the hardware.

Install renode at https://github.com/renode/renode/releases/

Run simulation

```bash
# From app directory
cd apps/<appname>
make sim

# OR From git root directory
make simulate BUILD=build/<appname>
```

Then run the simulation by either hooking into it with `gdb` via `make gdb`, or in the renode console type `start`.

If you do not see any logging output on the simulation, then you may be using the wrong logging backend. Try building with `LOG_UART=1` or run `make build-sim` in the app directory.

---

## How to build and host docs

### Install dependancies

```bash
# Install doxygen (Ubuntu/Debian)
sudo apt install doxygen graphviz
```

If on windows, install doxygen from [here](https://www.doxygen.nl/download.html), and graphviz from [here](https://graphviz.org/download/).

### Building and hosting the docs

```bash
# build docs
make docs

# host docs via localhost
# Will host it on `http://127.0.0.1:8080/`
make host
```

Pushing to the branches `master`, `unstable`, or `doctest` will trigger a GitHub Action, and will deploy the site to these URLS:

- https://ut-core-cubesat.github.io/ut-core-zephyr/stable
- https://ut-core-cubesat.github.io/ut-core-zephyr/unstable
- https://ut-core-cubesat.github.io/ut-core-zephyr/test



---

## Resources

### Zephyr Deep Dives

- [Zephyr's Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html)
- [Zephyr Documentation Home](https://docs.zephyrproject.org/latest/index.html)
- [West (Zephyr's meta-tool)](https://docs.zephyrproject.org/latest/develop/west/index.html)
- [Zephyr Kernel Services](https://docs.zephyrproject.org/latest/kernel/services/index.html)
- [Devicetree Guide](https://docs.zephyrproject.org/latest/build/dts/index.html)
- [CAN Bus support in Zephyr](https://docs.zephyrproject.org/latest/hardware/peripherals/can/index.html)


### Hardware Reference

- [STM32U5 Series (ST)](https://www.st.com/en/microcontrollers-microprocessors/stm32u5-series.html)
- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)


### Debugging & Simulation

- [OpenOCD User's Guide](http://openocd.org/doc/html/index.html)
- [GDB Documentation](https://sourceware.org/gdb/current/onlinedocs/gdb)
- [Renode Documentation](https://renode.readthedocs.io/)


### Git & Collaboration

- [Git Handbook (GitHub)](https://guides.github.com/introduction/git-handbook/)
- [Pro Git Book](https://git-scm.com/book/en/v2)
