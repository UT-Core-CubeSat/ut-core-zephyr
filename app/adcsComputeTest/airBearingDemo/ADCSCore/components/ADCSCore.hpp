#ifndef ADCSCORE_HPP
#define ADCSCORE_HPP

#include <core_Observer.hpp>
#include <core_ControllerManager.hpp>

namespace ADCS {

// COMMAND INTERFACE (What CDH/Ground sends to ADCS)
enum class MissionMode {
    OFF,      // No control, all actuators idle
    DETUMBLE, // BDot active (MTQ only, wheels idle)
    MOTOR,    // NDI attitude hold (reaction wheels only, no MTQ)
    BOTH      // NDI attitude hold + MTQ wheel desaturation
};

struct Command {
    MissionMode mode;
    Command() : mode(MissionMode::BOTH) {}
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
    Math::Vec<4> wheel_rpm;    // [RPM] desired wheel speeds (0 in OFF/DETUMBLE)
    Math::Vec<3> mtq_dipole;   // [A·m²] magnetorquer dipole (0 in OFF/MOTOR)
    Math::Vec<4> attitude_est;
    Math::Vec<3> rate_est;
    bool estimator_valid;
    MissionMode current_mode;
};

// UT-CORE-ADCS CLASS (The "Black Box")
class Core {
public: 
    Core();
    
    AdcsOutput update(const SensorData& sensors, const Command& command);
    AdcsOutput update(const SensorData& sensors);

    void reset();

    bool observerRunning() const { return observer_.isRunning(); }
    int  observerInitSamples() const { return observer_.getInitSamplesCollected(); }
    int  observerInitTarget() const { return ObserverClass::getInitSampleTarget(); }

private:
    ObserverClass observer_;
    ControllerManager controller_;

    Param::TimeReal last_time; // Store last update time 
    bool first_update; // Flag for first call
    
    // RTOS Optimization: Pre-allocated workspace to avoid per-call allocations
    // Reused across update() calls to minimize heap fragmentation and latency jitter
    mutable Param::Vector13 workspace_meas_;   // Measurement vector workspace

    // MOTOR/BOTH mode only arms after rates are sufficiently low.
    bool motor_mode_armed;
};

} // namespace ADCS

#endif // ADCSCORE_HPP