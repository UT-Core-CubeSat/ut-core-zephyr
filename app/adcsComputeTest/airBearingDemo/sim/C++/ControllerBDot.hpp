#ifndef CONTROLLERBDOT_HPP
#define CONTROLLERBDOT_HPP

#include "Parameters.hpp"
#include <cmath> 
#include "Saturate.hpp"
#include "HelperFunctions.hpp"

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
#endif // CONTROLLERBDOT_HPP