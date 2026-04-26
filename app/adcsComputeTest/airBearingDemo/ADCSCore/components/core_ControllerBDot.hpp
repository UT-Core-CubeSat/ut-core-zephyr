#ifndef CORE_CONTROLLERBDOT_HPP
#define CORE_CONTROLLERBDOT_HPP

#include "core_Parameters.hpp"
#include "core_Saturate.hpp"

class ControllerBDot {
public: 

    // Type Aliases for readability
    using StateVector = Param::Vector7;
    using Vector3 = Param::Vector3;
    using Vector4 = Param::Vector4;
    using Measurements = Param::Vector13;
    using Scalar = Param::Real;

    // Constructor
    ControllerBDot();

    // Update method - takes measurements and current estimated states
    // Marked inline: hot-path calculation in time-critical control loop
    Vector3 update(const Measurements& measurements, const Param::Vector11& states_hat, Scalar dt);
    
    Vector4 coil_currents_out;
    const Vector4& getCoilCurrents() const { return coil_currents_out; }
private:
    Scalar m_min, m_max;
    Scalar K_Bdot;
    Vector3 m_bang_state;
    Vector3 B_prev;
    Vector3 B_dot;
    Scalar alpha_Bdot;
    Vector3 Bdot_num_filt;
    Scalar beta_fuse;

    Vector4 dipoleToCoilCurrents(const Vector3& m_cmd);

    // Hysteresis state for coil activation (unipolar, so 4 channels)
    Vector4 coil_state; // init to Zero() in constructor 
};
#endif // CORE_CONTROLLERBDOT_HPP