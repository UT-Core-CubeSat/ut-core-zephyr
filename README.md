UT CubeSat - Zephyr RTOS Development

Custom STM32U5A5RJTXQ6Q board firmware for our CubeSat project using Zephyr RTOS.
Hardware

    MCU: STM32U5A5RJTXQ6Q
    Board: Custom UT CubeSat STM32U5 (ut_core)

Initial Setup
1. Follow the Official Zephyr Getting Started Guide

Follow the complete Zephyr Getting Started Guide for Windows:

https://docs.zephyrproject.org/latest/develop/getting_started/index.html

IMPORTANT: When you get to the "Get Zephyr and install Python dependencies" section, STOP after creating the virtual environment but before running west init. We'll use our custom setup instead.

So complete up to and including:
cmd

cd %HOMEPATH%
python -m venv zephyr-workspace\.venv
zephyr-workspace\.venv\Scripts\activate.bat
pip install west

2. Clone Our Repository Into the Workspace
cmd

cd %HOMEPATH%\zephyr-workspace
git clone https://github.com/braydonphillips/ut-core-zephyr.git ut-core

3. Initialize West with Our Repository

Instead of the guide's west init zephyrproject, run:
cmd

west init -l ut-core
west update

4. Continue with the Official Guide

Now continue with the official guide from "Export a Zephyr CMake package":
cmd

west zephyr-export
cmd /c scripts\utils\west-packages-pip-install.cmd
cd zephyr
west sdk install
cd ..

5. Set Board Root (Required for Our Custom Board)

This tells Zephyr where to find our ut_core board:
cmd

setx BOARD_ROOT %HOMEPATH%\zephyr-workspace\ut-core

Important: Close and reopen CMD, then reactivate the virtual environment:
cmd

cd %HOMEPATH%\zephyr-workspace
.venv\Scripts\activate.bat

Building Applications
Build an Application

From the zephyr-workspace directory with .venv activated:
cmd

west build -p always -b ut_core ut-core\app\<app-name>

Example apps:

    i2cTest - I2C peripheral test
    spiTest - SPI peripheral test

Flash to Board
cmd

west flash

Daily Workflow

Every time you open CMD:

    Navigate to workspace and activate virtual environment:

cmd

   cd %HOMEPATH%\zephyr-workspace
   .venv\Scripts\activate.bat

    Build/flash as needed

Troubleshooting
Board Not Found

If you get No board named 'ut_core' found:

    Make sure BOARD_ROOT is set by running the setx command in step 5
    Close and reopen CMD after setting it

