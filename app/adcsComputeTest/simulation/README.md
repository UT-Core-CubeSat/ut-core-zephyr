# Simulation Build Instructions

This folder contains the shared build entry point for the simulation project.
Use the `USE_ORBIT` CMake option to select between the two code bases:

- `OFF` (default): builds the air bearing demo from `../airBearingDemo/ADCSCore`
- `ON`: builds the orbit simulation from `../forOrbit/ADCSCore`


## All-in-one copy/paste (PowerShell)

### For Air Bearing Demo (default)
```powershell
Set-Location C:\Users\arick\zephyr-workspace\ut-core\app\adcsComputeTest
if (Test-Path simulation\build) { Remove-Item simulation\build -Recurse -Force }
New-Item -ItemType Directory -Path simulation\build | Out-Null
Set-Location simulation\build
cmake ..
cmake --build . --config Debug
.\Debug\PlantSim.exe
Set-Location ..
```
### For small changes 
```powershell
Set-Location C:\Users\arick\Desktop\ut-core-controls\simulation\build
cmake --build . --config Debug
.\Debug\PlantSim.exe
```

### For Orbit Simulation
```powershell
Set-Location C:\Users\arick\Desktop\ut-core-controls
if (Test-Path simulation\build) { Remove-Item simulation\build -Recurse -Force }
New-Item -ItemType Directory -Path simulation\build | Out-Null
Set-Location simulation\build
cmake .. -DUSE_ORBIT=ON
cmake --build . --config Debug
.\Debug\PlantSim.exe
```

## Same flow in cmd.exe (not PowerShell)

### Air Bearing Demo (default)
```bat
cd /d C:\Users\arick\Desktop\ut-core-controls
rmdir /s /q simulation\build
mkdir simulation\build
cd simulation\build
cmake ..
cmake --build . --config Debug
Debug\PlantSim.exe
```

### Orbit Simulation
```bat
cd /d C:\Users\arick\Desktop\ut-core-controls
rmdir /s /q simulation\build
mkdir simulation\build
cd simulation\build
cmake .. -DUSE_ORBIT=ON
cmake --build . --config Debug
Debug\PlantSim.exe
```

## Build steps

1. Create and enter a build directory (from the `ut-core-controls` root):

```powershell
Set-Location C:\Users\arick\Desktop\ut-core-controls
if (-not (Test-Path simulation\build)) { New-Item -ItemType Directory -Path simulation\build | Out-Null }
Set-Location simulation\build
```

2. Configure the project:

- Air bearing demo (default):

```bash
cmake ..
```

- Orbit simulation:

```bash
cmake .. -DUSE_ORBIT=ON
```

3. Build the executable:

```bash
cmake --build .
```

## Run the simulation

After building, run the shared `PlantSim` executable:

**On Windows (Visual Studio generator):**
```powershell
# From simulation/build/
.\Debug\PlantSim.exe
```

**On Linux/macOS:**
```bash
# From simulation/build/
./PlantSim
```

**Note:** The executable location depends on your CMake generator and build type:
- Visual Studio (Windows): `simulation/build/Debug/PlantSim.exe` (or `Release/` if built in Release mode)
- Unix Makefiles: `simulation/build/PlantSim`

## Notes

- The build uses the source code from either `airBearingDemo/ADCSCore` or `forOrbit/ADCSCore`.
- `plant_main.cpp` is compiled into the `PlantSim` executable during the build.
- If you switch between scenarios, rerun CMake in the same build directory with the desired `USE_ORBIT` setting, or delete the build directory and recreate it.
