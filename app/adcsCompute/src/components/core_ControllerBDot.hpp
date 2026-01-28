#ifndef CORE_CONTROLLERBDOT_HPP
#define CORE_CONTROLLERBDOT_HPP

#include "core_Parameters.hpp"
#include "core_Saturate.hpp"
#include "core_HelperFunctions.hpp"

class ControllerBDot {
public: 

    // Output Struct
    struct BDotOutput {
        Param::Vector3 tau_sat;
        Param::Vector7 states_m;
    };

    // Type Aliases for readability
    using StateVector = Param::Vector7;
    using Vector3 = Param::Vector3;
    using Measurements = Param::Vector29;
    using Scalar = Param::Real;
    using NaN = decltype(std::nan(""));

    // Constructor
    ControllerBDot();

    // Update method - takes measurements and current estimated states
    BDotOutput update(const Measurements& measurements, const Param::Vector17& states_hat);
    
private:
    // Private Members 
    HelperFunctions helpers;
    Scalar m_min, m_max;
    Scalar K_Bdot;
    Vector3 B_prev;
    Scalar dt;
    Vector3 B_dot;
    Scalar alpha_Bdot;
    Vector3 Bdot_num_filt;
    Scalar beta_fuse;
};
#endif // CORE_CONTROLLERBDOT_HPP