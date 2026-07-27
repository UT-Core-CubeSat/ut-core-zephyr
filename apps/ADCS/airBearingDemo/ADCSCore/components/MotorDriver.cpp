#include "MotorDriver.hpp"

MotorDriver::MotorDriver() {
    // Initialize parameters
    k_p = 1.0;
    k_d = 0.1;
    I_wheel = 1.13e-6;
    M_PI = 3.14159265358979323846; // Define pi if not defined in math library
    b = Vector4::Constant(0.0); // Intercept for duty cycle mapping (to be tuned)
    m = Vector4::Constant(1.0); // Slope for duty cycle mapping
    omega_w_max = Param::Actuators::omega_w_max;
    prev_omega_w = Vector4::Zero(); // Initialize previous wheel speeds to zero
    omega_target = Vector4::Zero(); // Initialize target wheel speeds to zero
    last_mode = MissionMode::OFF;
}

MotorDriver::Vector4 MotorDriver::computeMotorCommands(const Vector4& tau_w_cmd, const Vector4& omega_w, const Scalar& dt, const MissionMode& mission_mode) {
   // Safety, state machine 
   if (mission_mode == ADCS::MissionMode::OFF) {
    omega_target = omega_w;
    last_mode = ADCS::MissionMode::OFF;
    return Vector4::Zero(); // No control in OFF mode
   }

   // Mode transition latch 

   if (mission_mode != last_mode) {
    omega_target = omega_w;
    prev_omega_w = omega_w;
    last_mode = mission_mode;
   }
   
    // PD control law to convert desired wheel torques to 4 duty cycle commands, based on the experimentally 
    // found relationship between duty cycle and RPM's for our specific reaction wheels. 

    omega_target = omega_target + (tau_w_cmd / I_wheel) * dt;
    // Saturate target wheel speeds to max limits
    omega_target = saturateSymmetric(omega_target, omega_w_max);

    // Define the signals for PD 
    Vector4 error = omega_target - omega_w; 
    Vector4 target_accel = tau_w_cmd / I_wheel; 
    Vector4 measured_accel = (omega_w - prev_omega_w) / dt; 
    // Pass measured_accel through a simple LPF before PD 
    // For simplicity, we can implement a first-order low-pass filter on the measured acceleration
    static const Scalar alpha = 0.5; // Smoothing factor for the low-pass filter (tune as needed)
    static Vector4 accel_lpf = Vector4::Zero(); // Initialize the filtered acceleration
    accel_lpf = alpha * measured_accel + (1 - alpha) * accel_lpf; // Update the filtered acceleration
    measured_accel = accel_lpf; // Use the filtered acceleration for the PD control

    // Base duty cycle (feed forward from RPM mapping)
    Vector4 omega_target_rpm = omega_target * (60.0 / (2.0 * M_PI)); // Convert rad/s to RPM
    Vector4 duty_cycle_base; 
    for (int i = 0; i < 4; ++i) {
        duty_cycle_base(i) = (omega_target_rpm(i) - b(i)) / m(i); // Invert the linear mapping
    }

    // PD feedback + accel feedforward 
    Vector4 duty_cycle_pd = duty_cycle_base + k_p * error + k_d * (target_accel - measured_accel);

    prev_omega_w = omega_w;

    return saturateSymmetric(duty_cycle_pd*100, static_cast<Scalar>(1.0)); // Saturate duty cycle to [-100, 100]
}