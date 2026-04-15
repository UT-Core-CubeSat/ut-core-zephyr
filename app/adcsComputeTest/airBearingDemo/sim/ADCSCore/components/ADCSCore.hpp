#ifndef ADCSCORE_HPP
#define ADCSCORE_HPP

#include <core_Observer.hpp>
#include <core_ControllerManager.hpp>

namespace ADCS {

// ============================================================================
// SENSOR INTERFACE (What CAN provides from sensors)
// ============================================================================
struct SensorData {
    Param::TimeReal unix_time;          // From GPS or RTC
    Math::Vec<3> accelerometer;   // [m/s^2] gravity vector in body frame
    Math::Vec<3> gyro;         // [rad/s] body rates
    Math::Vec<3> magnetometer; // [T] magnetic field in body frame
    Math::Vec<4> wheel_speeds; // [rad/s] reaction wheel angular velocities
};

// ============================================================================
// OUTPUT INTERFACE (What ADCS sends back via CAN)
// ============================================================================
struct AdcsOutput {
    Math::Vec<4> wheel_torque;
    Math::Vec<3> mtq_dipole;
    Math::Vec<4> attitude_est;
    Math::Vec<3> rate_est;
    bool estimator_valid;
    
    // Add these for logging equivalence. These won't be here later
    Param::Vector10 reference;   // Reference trajectory
    Param::Vector7 states_m;     // Model states from controller
    Param::Vector11 states_hat;  // Full estimated state
};

// ============================================================================
// CORE CLASS (The Black Box)
// ============================================================================
class Core {
public: 
    Core();
    
    AdcsOutput update(const SensorData& sensors);
    
    void reset();

private:
    ObserverClass observer_;
    ControllerManager controller_;

    Param::TimeReal last_time; // Store last update time 
    bool first_update; // Flag for first call
    Param::PointingMode mode;
};

} // namespace ADCS

#endif // ADCSCORE_HPP