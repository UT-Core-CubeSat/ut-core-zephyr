#ifndef ADCSCORE_HPP
#define ADCSCORE_HPP

#include <components/core_Observer.hpp>
#include <components/core_ReferenceGenerator.hpp>
#include <components/core_ControllerManager.hpp>

namespace ADCS {

// ============================================================================
// COMMAND INTERFACE (What CDH/Ground sends to ADCS)
// ============================================================================

// Mission-level modes (what operators command)
enum class MissionMode { 
    SAFE,       // Power down
    DETUMBLE,  // Detumble, minimal actuator use
    STANDBY,    // Sun-pointing for power generation
    DOWNLINK,   // Point antenna at ground station
    IMAGING,    // Point boresight at earth target
    CUSTOM      // Escape hatch for explicit vector control
};

// Ground target for geo-pointing modes
struct GroundTarget {
    double latitude;   // [deg] geodetic
    double longitude;  // [deg]
    double altitude;   // [m] above WGS84 ellipsoid (0 for ground stations)
};

// Command from CDH/Ground
struct Command {
    MissionMode mode;
    
    // Used for DOWNLINK, IMAGING modes
    GroundTarget target;
    
    // For IMAGING: true = track target location, false = point nadir
    bool track_target;
    
    // For CUSTOM mode only (escape hatch)
    Math::Vec<3> body_axis;
    Math::Vec<3> target_eci;
    
    // Helper: default constructor
    Command() : mode(MissionMode::SAFE), target{0,0,0}, track_target(false),
                body_axis{0,0,1}, target_eci{1,0,0} {}
};

// ============================================================================
// SENSOR INTERFACE (What CAN provides from sensors)
// ============================================================================
struct SensorData {
    double unix_time;          // From GPS or RTC
    Math::Vec<3> gyro;         // [rad/s] body rates
    Math::Vec<4> star_quat;    // [q0,q1,q2,q3] quaternion, scalar-first
    Math::Vec<6> css_currents; // [A] coarse sun sensor currents (6 faces)
    Math::Vec<3> magnetometer; // [T] magnetic field in body frame
    Math::Vec<6> gps_ecef;     // [m, m/s] position and velocity in ECEF
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
    MissionMode current_mode;
    
    // Add these for logging equivalence. These won't be here later
    Param::Vector10 reference;   // Reference trajectory
    Param::Vector7 states_m;     // Model states from controller
    Param::Vector17 states_hat;  // Full estimated state
};

// ============================================================================
// CORE CLASS (The Black Box)
// ============================================================================
class Core {
public: 
    Core();
    
    AdcsOutput update(const SensorData& sensors, const Command& command);
    
    void reset();

private:
    ObserverClass observer_;
    ReferenceGenerator refgen_;
    ControllerManager controller_;
};

} // namespace ADCS

#endif // ADCSCORE_HPP