#ifndef ADCSCORE_HPP
#define ADCSCORE_HPP

#include <core_Observer.hpp>
#include <core_ControllerManager.hpp>
#include <core_MTQAllocator.hpp>

namespace ADCS {

// COMMAND INTERFACE (What CDH/Ground sends to ADCS)
enum class MissionMode {
    OFF,       // No control
    SAFE,      // B-dot active
    BEARING    // NDI attitude hold + wheel desat
};

struct Command {
    MissionMode mode;
    Command() : mode(MissionMode::BEARING) {}
};

// SENSOR INTERFACE (What CAN provides from sensors)
struct SensorData {
    Param::TimeReal unix_time;          // From GPS or RTC
    Math::Vec<3> accelerometer;   // [m/s^2] gravity vector in body frame
    Math::Vec<3> gyro;         // [rad/s] body rates
    Math::Vec<3> magnetometer; // [T] magnetic field in body frame
    Math::Vec<4> wheel_speeds; // [rad/s] reaction wheel angular velocities
};

// OUTPUT INTERFACE (What ADCS sends back via CAN)
struct AdcsOutput {
    Math::Vec<4> wheel_torque;
    Math::Vec<3> mtq_dipole;
    Math::Vec<4> attitude_est;
    Math::Vec<3> rate_est;
    bool estimator_valid;
    MissionMode current_mode;
    
    // Per-Face Magnetorquer Derived Outputs (auto-populated from mtq_dipole)
    // Software engineer uses these directly for CAN without needing control law knowledge
    Math::Vec<4> mtq_face_current;        // [A] Per-face current: [I_Xpos, I_Xneg, I_Ypos, I_Yneg]
    Math::Vec<4> mtq_face_b_ref;          // [T] Per-face reference field magnitude produced at face
    
    // These are for logging equivalence. Won't be here in orbit, but they make it easier to plot and debug in simulation.
    Param::Vector10 reference;   // Reference trajectory
    Param::Vector7 states_m;     // Model states from controller
    Param::Vector11 states_hat;  // Full estimated state
    
    // Innovation diagnostics (for tuning)
    Math::Vec<3> accel_innovation;
    Math::Real accel_innovation_norm;
    Math::Vec<3> mag_innovation;
    Math::Real mag_innovation_norm;
};

// UT-CORE-ADCS CLASS (The "Black Box")
class Core {
public: 
    Core();
    
    AdcsOutput update(const SensorData& sensors, const Command& command);
    AdcsOutput update(const SensorData& sensors);
    
    void reset();

private:
    ObserverClass observer_;
    ControllerManager controller_;
    MTQAllocator mtq_allocator_;  // Per-face magnetorquer current and field allocator

    Param::TimeReal last_time; // Store last update time 
    bool first_update; // Flag for first call
    
    // RTOS Optimization: Pre-allocated workspace to avoid per-call allocations
    // Reused across update() calls to minimize heap fragmentation and latency jitter
    mutable Param::Vector13 workspace_meas_;   // Measurement vector workspace
    mutable Param::Vector10 workspace_ref_;    // Reference workspace

    // BEARING mode only arms after rates are sufficiently low.
    bool bearing_mode_armed;
};

} // namespace ADCS

#endif // ADCSCORE_HPP