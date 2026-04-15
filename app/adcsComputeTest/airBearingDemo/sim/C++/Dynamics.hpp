#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include "Parameters.hpp"
#include "HelperFunctions.hpp"
#include "Saturate.hpp"

class Dynamics {
public: 

    using InputVector = Param::Vector7;
    using StateVector = Param::Vector7;
    using RefVector = Param::Vector10;
    using Scalar = Param::Real;
    using Vector3 = Param::Vector3;
    using Vector4 = Param::Vector4;
    using Quat = Param::Vector4;

    // Constructor 
    Dynamics();
    void update(const InputVector& inputs, const Param::PointingMode mode);
    const Param::Vector17& getStates() const { return states; }
    const Param::Vector17& getStatesDot() const { return states_dot; }

private: 
    // Helper Functions 
    HelperFunctions helpers; 

    // Private Methods 
    void rk4_kinetics(const InputVector& inputs, const Param::PointingMode mode);

    Param::Vector17 kinetics(const Param::Vector17& states, const InputVector& inputs, const Param::PointingMode mode);

    struct forceOutput {
        Vector3 forces;
        InputVector moments;
    };

    forceOutput forces_and_moments(const InputVector& inputs);

    // Private Members
    Scalar Ts;
    Scalar mu_E;
    Scalar r_E;
    Param::Matrix3 I;
    Param::Matrix3 I_inv;
    Scalar G;
    Param::Vector17 states;
    RefVector reference;
    Param::Vector17 states_dot;
    Param::Matrix34 S;
    Scalar I_wheel;
};

#endif // DYNAMICS_HPP