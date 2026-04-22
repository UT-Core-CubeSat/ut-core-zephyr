#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include "Plant_Parameters.hpp"
#include "../components/core_HelperFunctions.hpp"
#include "../components/core_Saturate.hpp"

class Dynamics {
public: 
    using InputVector = Param::Vector7;
    using StateVector = Param::Vector11;
    using RefVector = Param::Vector10;
    using Scalar = Param::Real;
    using Vector3 = Param::Vector3;
    using Vector4 = Param::Vector4;
    using Quat = Param::Vector4;

    Dynamics();
    void update(const InputVector& inputs);
    const Param::Vector11& getStates() const { return states; }
    const Param::Vector11& getStatesDot() const { return states_dot; }
    
    // Expose disturbance for logging
    Vector3 getTauGravity() const { return tau_gravity_current; }
    Vector3 getTauDisturbance() const { return tau_disturbance_current; }

private: 
    HelperFunctions helpers; 

    void rk4_kinetics(const InputVector& inputs);
    Param::Vector11 kinetics(const Param::Vector11& states, 
                              const InputVector& inputs,
                              const Scalar& t);

    struct forceOutput {
        Vector3 forces;
        InputVector moments;
        Vector3 tau_gravity;
        Vector3 tau_disturbance;
        Vector3 tau_friction;
    };

    forceOutput forces_and_moments(const InputVector& inputs,
                                    const Quat& q,
                                    const Vector3& omega,
                                    const Scalar& t);
    
    // Wheel internal dynamics helpers 
    Vector4 compute_motor_current_dynamics(const Vector4& V_cmd, const Vector4& omega_wheel, const Vector4& i_motor);
    Vector4 compute_wheel_friction(const Vector4& omega_wheel) const;
    Vector4 compute_wheel_ripple(const Vector4& omega_wheel) const;

    // Members
    Scalar Ts;
    Param::Matrix3 I;
    Param::Matrix3 I_inv;
    Param::Vector11 states;
    RefVector reference;
    Param::Vector11 states_dot;
    Param::Matrix34 S;
    Scalar I_wheel;
    
    // Air bearing specific
    Scalar h_cg;          // CG offset from pivot [m]
    Scalar m;             // Mass [kg]
    Scalar g;             // Gravity [m/s^2]
    Scalar r_dist;        // Disturbance arm [m]
    Scalar b_friction;    // Bearing friction [Nm·s/rad]
    Scalar t_current;     // Current simulation time
    
    // For logging
    Vector3 tau_gravity_current;
    Vector3 tau_disturbance_current;

    // Wheel motor internal dynamics state
    Vector4 i_motor;           // Motor phase current [A]
    Vector4 ripple_phase;      // Ripple phase [rad]
};

#endif // DYNAMICS_HPP