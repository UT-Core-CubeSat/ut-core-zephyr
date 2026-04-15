# Utah Tech Harbinger CubeSat Control Code

## Overview
This repository contains the attitude determination and control system (ADCS) software for the Utah Tech Harbinger CubeSat mission. The code implements a sophisticated control system using quaternion-based attitude representation, model reference adaptive control, and sensor fusion via Extended Kalman Filter (EKF) for precise orientation control in the space environment.

## Mission Description
The Harbinger CubeSat is a mission developed by Utah Tech University aimed at taking thermal images of Utah. The ADCS system enables precision pointing required for pointing accuracy and communications to the ground station.

## System Architecture
The control system is organized into the following modules:

- **Controller**: Implements LQR (Linear Quadratic Regulator) control with gain scheduling for different attitude regions
- **Observer**: Implements an Extended Kalman Filter (EKF) for state estimation based on sensor measurements
- **Sensors**: Manages gyroscope and star tracker inputs with noise modeling
- **Dynamics**: Simulates CubeSat rotational dynamics including reaction wheel actuation
- **Simulation**: Provides environment for testing the complete system with visualization

## Hardware Components
- **Sensors**:
  - 3-axis MEMS gyroscope
  - Star tracker (quaternion output)
  - [Future expansion for magnetometer support]
- **Actuators**:
  - 4 reaction wheels in tetrahedral configuration
- **Processor**:
  - [Processor specifications]

## Control System Design
### State Representation
- Full state vector: `[q1, q2, q3, q4, ωx, ωy, ωz]`
  - Quaternion attitude representation `[q1, q2, q3, q4]`
  - Angular velocity vector `[ωx, ωy, ωz]`
- Modified Rodrigues Parameters (MRP) used internally for control

### Controller Features
- Gain-scheduled LQR control
- Non-zero setpoint tracking for quaternion references
- Reaction wheel control allocation
- Automatic saturation handling (RPM limiting)

### Observer (State Estimation)
- Extended Kalman Filter implementation
- Sensor fusion between gyroscope and star tracker
- Asynchronous star tracker updates (lower frequency)
- Fourth-order Runge-Kutta integration for state propagation

## Mathematics
The controller uses several key mathematical transformations:
- Quaternion error calculation: `qₑ = q₁⁻¹ ⊗ q₂`
- Quaternion to Modified Rodrigues Parameters (MRP) conversion
- State space representation of attitude dynamics
- LQR optimal control design

## Configuration
Key parameters include:

- `Q`, `R`: LQR cost matrices
- `J`: Satellite moment of inertia tensor
- `gains`: Pre-computed gain table for different operating points
- `sigma_gyro_x/y/z`: Gyroscope noise parameters
- `sigma_star_quaternion`: Star tracker noise parameter
- `Ts_star_tracker`: Star tracker update period

## Usage
```matlab
% Setup system
parameters = setupParameters();  % Define all system parameters
observer = Observer(parameters);
controller = Controller(parameters);
dynamics = Dynamics(parameters);
sensors = Sensors(parameters);

% Main control loop
for t = 0:dt:simulation_time
    % Get sensor measurements
    measurements = sensors.update(state, magnetic_field, t, false);
    
    % Estimate state with EKF
    state_estimate = observer.update(control_input, measurements);
    
    % Calculate control input
    control_input = controller.update(state_estimate, reference);
    
    % Update system dynamics
    state = dynamics.update(state, control_input, dt);
    
    % Visualization updates
    visualization.update(state);
end
```

## Code Structure
```
├── Controller.m      # LQR controller implementation
├── Observer.m        # Extended Kalman Filter implementation
├── Sensors.m         # Sensor models with noise
├── Dynamics.m        # Satellite dynamics simulation 
├── Simulation.m      # Main simulation loop
├── Animation.m       # 3D visualization
├── Parameters.m      # System parameter definitions
├── Plotter.m         # Data visualization
└── Utilities/        # Helper functions
    ├── quat2MRP.m    # Quaternion to MRP conversion
    ├── MRP2quat.m    # MRP to quaternion conversion
    ├── tau2RPM.m     # Torque to reaction wheel RPM conversion
    ├── RPM2tau.m     # RPM to torque conversion
    ├── tau2PWM.m     # Torque to PWM conversion
    └── cross_mat.m   # Cross product matrix
```

## Features
- **Quaternion-Based Control**: Full 3D attitude representation without singularities
- **Modified Rodrigues Parameters**: Used for efficient control implementation
- **Gain Scheduling**: Adaptation to different operating regimes
- **Sensor Fusion**: Integration of multiple sensor inputs for optimal state estimation
- **Reaction Wheel Control**: Precise torque allocation for 3-axis control
- **3D Visualization**: Real-time display of satellite attitude
- **Telemetry Plotting**: Comprehensive data visualization

## Testing
The system includes comprehensive simulation capabilities:
- Full 3D visualization of satellite attitude
- Time-domain response plotting
- Reference tracking performance analysis
- Sensor noise and disturbance rejection testing

## Flight Modes
The system implements several operational modes:
1. **Detumble Mode**: Initial stabilization after deployment
2. **Pointing Mode**: Precision attitude maintenance for mission operations
3. **Slew Mode**: Large angle maneuvers between target orientations

## Future Enhancements
- Magnetometer integration for additional heading reference
- Momentum management strategies for reaction wheel desaturation
- Improved disturbance rejection for external torques
- Fault detection and recovery logic

## Contact
- Benjamin Sherman benjaminsherman18@gmail.com

## Acknowledgments
- Utah Tech University
- Utah NASA Space Grant Consortium
- Utah Division of Natural Resources
- OnShape
- Washington County Water Conservancy District
- RAM
- Dr Kameron Eves (He made this all possible, go to him with any questions)

---

## Version History
- v0.1.0: Initial ADCS implementation