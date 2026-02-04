#include "core_ControllerBDot.hpp"
#include "core_Saturate.hpp"
// Constructor 
ControllerBDot::ControllerBDot()
    : // Initialize Members 
    m_min(Param::Actuators::m_min),
    m_max(Param::Actuators::m_max),
    K_Bdot(Param::Controller::K_Bdot),
    B_prev(Vector3::Zero()),
    dt(Param::SimTime::Ts),
    B_dot(Vector3::Zero()),
    alpha_Bdot(Param::Controller::alpha_BDot),
    Bdot_num_filt(Vector3::Zero()),
    beta_fuse(Param::Controller::beta_fuse)
{
    // Constructor body (if needed)
}

// Methods
ControllerBDot::BDotOutput ControllerBDot::update(const Measurements& measurements, const Param::Vector17& states_hat)
{
    // Extract magnetic field measurement
    Vector3 B_now = measurements.segment<3>(16); // Assuming B field is in elements 16-18
    Vector3 omega_meas = measurements.segment<3>(3); // Assuming angular velocity is in elements 3-5
    Vector3 m_tilde;
    Vector3 Bh;
    Param::Matrix3 P_perp;
    Vector3 m_sat;
    
    // Build states_m from current estimated states (q and omega)
    // states_hat: [R(3), V(3), q(4), omega(3), omega_w(4)]
    // states_m format: [q(4), omega(3)] with q as [q0, q1, q2, q3]
    StateVector states_m;
    //states_m << states_hat.segment<4>(6), states_hat.segment<3>(10);
    states_m(0) = states_hat(6); // q0
    states_m(1) = states_hat(7); // q1
    states_m(2) = states_hat(8); // q2
    states_m(3) = states_hat(9); // q3
    states_m(4) = states_hat(10); // omega_x
    states_m(5) = states_hat(11); // omega_y
    states_m(6) = states_hat(12); // omega_z

    if (B_prev.isZero()) {
        B_prev = B_now;
        B_dot = Vector3::Zero();
        return {BDotOutput{Vector3::Zero(), states_m}};
    }
    // Apply low-pass filter to B_dot
    Bdot_num_filt = (1 - alpha_Bdot)*(B_now - B_prev) / dt + alpha_Bdot * Bdot_num_filt;
    B_dot = (1-beta_fuse)*(-omega_meas.cross(B_now)) + beta_fuse*Bdot_num_filt;
    m_tilde = -K_Bdot*B_dot;
    // Saturate magnetic moment
    if (B_now.norm() > 1e-9) {
        Bh = B_now / B_now.norm();

        // 1. Explicitly calculate the outer product into a Matrix3
        Param::Matrix3 outerProduct = Math::outerProduct(Bh, Bh);

        // 2. Perform the subtraction from Identity
        P_perp = Param::Matrix3::Identity() - outerProduct;

        m_tilde = P_perp * m_tilde;
    }
    
    m_sat = saturateSymmetric(m_tilde, m_max);
    // Compute torque
    Vector3 tau_sat = m_sat.cross(B_now);
    // Update B_prev
    B_prev = B_now;
    return {BDotOutput{tau_sat, states_m}};
}