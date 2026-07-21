#include "Dynamics.hpp" 
#include <iostream> // For debugging purposes
#include "two_body.hpp"
// Constructor 
Dynamics::Dynamics()
    : Ts(PlantParam::SimTime::Ts),
      mu_E(PlantParam::Earth::mu_E),
      r_E(PlantParam::Earth::r_E),
      I(PlantParam::Spacecraft::I),
      I_inv(Math::inverse3x3(PlantParam::Spacecraft::I)),
      G(PlantParam::Earth::G),
      states(PlantParam::Vector17::Zero()),
      reference(PlantParam::Vector10::Zero()),
      states_dot(PlantParam::Vector17::Zero()),
      S(PlantParam::Actuators::S),
      I_wheel(PlantParam::Actuators::I_wheel)
{
    states.setSegment(0, PlantParam::Orbit::InitialState.r_ECI);
    states.setSegment(3, PlantParam::Orbit::InitialState.v_ECI);
    states.setSegment(6, PlantParam::Orbit::InitialState.q0);
    states.setSegment(10, PlantParam::Orbit::InitialState.omega);
    states.setSegment(13, PlantParam::Orbit::InitialState.omega_wheel);
}

void Dynamics::update(const InputVector& inputs) {
    rk4_kinetics(inputs);
}

void Dynamics::rk4_kinetics(const InputVector& inputs) {
    // RK4 Integration 
    PlantParam::Vector17 k1 = kinetics(states, inputs);
    PlantParam::Vector17 k2 = kinetics(states + (Ts / static_cast<Scalar>(2.0)) * k1, inputs);
    PlantParam::Vector17 k3 = kinetics(states + (Ts / static_cast<Scalar>(2.0)) * k2, inputs);
    PlantParam::Vector17 k4 = kinetics(states + Ts * k3, inputs);

    states = states + (Ts / static_cast<Scalar>(6.0)) * (k1 + static_cast<Scalar>(2.0) * k2 + static_cast<Scalar>(2.0) * k3 + k4);
    PlantParam::Quat q_norm;
    q_norm(0) = states(6);
    q_norm.setSegment(1, states.segment<3>(7));
    q_norm.normalize();
    states.setSegment(6, q_norm);
    states.setSegment(13, saturateSymmetric<4>(states.segment<4>(13), PlantParam::Actuators::omega_w_max));
}

PlantParam::Vector17 Dynamics::kinetics(const PlantParam::Vector17& states, const InputVector& inputs) {
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
    Vector3 q_dot_vec = static_cast<Scalar>(0.5)*(q0*omega + q_vec.cross(omega));
    Scalar q0_dot = static_cast<Scalar>(-0.5) * q_vec.dot(omega);

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