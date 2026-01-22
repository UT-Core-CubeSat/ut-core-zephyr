UT CubeSat - Zephyr RTOS Development

Custom STM32U5A5RJTXQ6Q board firmware for our CubeSat project using Zephyr RTOS.
Hardware

    MCU: STM32U5A5RJTXQ6Q
    Board: Custom UT CubeSat STM32U5 (ut_core)

Initial Setup
1. Follow the Official Zephyr Getting Started Guide

Follow the complete Zephyr Getting Started Guide for Windows:

https://docs.zephyrproject.org/latest/develop/getting_started/index.html

Complete ALL steps in the guide including:

    Installing dependencies
    Creating virtual environment
    Running west init zephyr-workspace
    Running west update
    Installing Python dependencies
    Installing Zephyr SDK

2. Clone Our Repository Into the Workspace

After completing the official guide, clone our custom board and apps:
cmd

cd %HOMEPATH%\zephyr-workspace
git clone https://github.com/braydonphillips/ut-core-zephyr.git ut-core

3. Set Board Root (Required for Our Custom Board)

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

