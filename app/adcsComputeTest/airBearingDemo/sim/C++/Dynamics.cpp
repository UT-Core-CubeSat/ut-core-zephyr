#include "Dynamics.hpp" 
#include <iostream> // For debugging purposes
#include "two_body.hpp"
// Constructor 
Dynamics::Dynamics()
    : Ts(Param::SimTime::Ts),
      mu_E(Param::Earth::mu_E),
      r_E(Param::Earth::r_E),
      I(Param::Spacecraft::I),
      I_inv(Math::inverse3x3(Param::Spacecraft::I)),
      G(Param::Earth::G),
      states(Param::Vector17::Zero()),
      reference(Param::Vector10::Zero()),
      states_dot(Param::Vector17::Zero()),
      S(Param::Actuators::S),
      I_wheel(Param::Actuators::I_wheel)
{
    states.setSegment(0, Param::Orbit::InitialState.r_ECI);
    states.setSegment(3, Param::Orbit::InitialState.v_ECI);
    states.setSegment(6, Param::Orbit::InitialState.q0);
    states.setSegment(10, Param::Orbit::InitialState.omega);
    states.setSegment(13, Param::Orbit::InitialState.omega_wheel);
}

void Dynamics::update(const InputVector& inputs, const Param::PointingMode mode) {
    rk4_kinetics(inputs, mode);
}

void Dynamics::rk4_kinetics(const InputVector& inputs, const Param::PointingMode mode) {
    // RK4 Integration 
    Param::Vector17 k1 = kinetics(states, inputs, mode);
    Param::Vector17 k2 = kinetics(states + (Ts / 2.0) * k1, inputs, mode);
    Param::Vector17 k3 = kinetics(states + (Ts / 2.0) * k2, inputs, mode);
    Param::Vector17 k4 = kinetics(states + Ts * k3, inputs, mode);

    states = states + (Ts / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    Param::Quat q_norm;
    q_norm(0) = states(6);
    q_norm.setSegment(1, states.segment<3>(7));
    q_norm.normalize();
    states.setSegment(6, q_norm);
    states.setSegment(13, saturateSymmetric<4>(states.segment<4>(13), Param::Actuators::omega_w_max));
}

Param::Vector17 Dynamics::kinetics(const Param::Vector17& states, const InputVector& inputs, const Param::PointingMode mode) {
    // Unpack States 
    Vector3 r_ECI = states.segment<3>(0);
    Vector3 v_ECI = states.segment<3>(3);
    Quat q = states.segment<4>(6);
    Vector3 omega = states.segment<3>(10);
    Vector4 omega_wheel = states.segment<4>(13);

    // Calculate forces and moments 
    forceOutput fm = forces_and_moments(inputs);
    Vector4 tau_rw = fm.moments.segment<4>(0);
    Vector3 tau_mtq = fm.moments.segment<3>(4);

    Vector3 h_w = S * (I_wheel * omega_wheel); // Reaction Wheel Angular Momentum
    Vector3 tau_ext = tau_mtq - (S * tau_rw);

    Vector3 R_dot = v_ECI;

    Vector3 V_dot = TwoBody::two_body(mu_E, r_E, r_ECI);

    Vector3 q_vec = q.segment<3>(1);
    Scalar q0 = q(0);
    Vector3 q_dot_vec = 0.5*(q0*omega + q_vec.cross(omega));
    Scalar q0_dot = -0.5 * q_vec.dot(omega);

    Quat q_dot; 
    q_dot(0) = q0_dot;
    q_dot.setSegment(1, q_dot_vec);
    Vector3 omega_dot = I_inv * (tau_ext - omega.cross(I * omega + h_w));

    Vector4 omega_wheel_dot = tau_rw / I_wheel;

    states_dot.setSegment(0, R_dot);
    states_dot.setSegment(3, V_dot);
    states_dot.setSegment(6, q_dot);
    states_dot.setSegment(10, omega_dot);
    states_dot.setSegment(13, omega_wheel_dot);
    return states_dot;
}

Dynamics::forceOutput Dynamics::forces_and_moments(const InputVector& inputs) {
    // Unpack Inputs 
    InputVector moments = inputs;

    return forceOutput{Vector3::Zero(), moments};
}