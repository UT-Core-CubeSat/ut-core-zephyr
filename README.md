# UT CubeSat - Zephyr RTOS Development

Custom STM32U5A5RJTXQ6Q board firmware for our CubeSat project using Zephyr RTOS.
Hardware

**MCU:** STM32U5A5RJTXQ6Q

**Board:** Custom UT CubeSat STM32U5 (ut_core)


<!-- For testing the installation instructions on a new debian system -->
<!-- sudo docker run -it --rm -v $(pwd):/mnt debian:latest bash -->


## Project Structure

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



## How to Build

(Only tested on Debian)


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


### Install dependancies (Windows)

(TBD)


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


## How to view board logs

Whilst connected to the board with the ST-Link cable, start a debug server over the link in the first terminal by running `make debug` or `openocd -f tools/openocd_semihost.cfg`. In a second terminal start a telnet client by running `make listen` or `telnet localhost 9090`, which if successful will print out the debug logs.

```bash
# in the first terminal
make debug

# in a second terminal
make listen
```


## How to run GDB debugger

Whilst connected to the board with the ST-Link cable, start a debug server over the link in the first terminal by running `make debug` or `openocd -f tools/openocd_semihost.cfg`. In a second terminal start a gdb client with `make gdb APP=apps/<appname>`.

```bash
# in the first terminal
make debug

# in a second terminal from the app directory
cd app/<appname>
make gdb

# OR from the git root directory
make gdb BUILD=build/<appname>
```


## How to simulate boards

**CAUTION** - Full simulation is not implemented!

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


## Resources

For more information on Zephyr, 
https://docs.zephyrproject.org/latest/develop/getting_started/index.html

Zephyr docs
https://docs.zephyrproject.org/latest/index.html

