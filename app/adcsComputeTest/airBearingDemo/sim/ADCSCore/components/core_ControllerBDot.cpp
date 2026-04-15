#include "core_ControllerBDot.hpp"
#include "core_Saturate.hpp"
// Constructor 
ControllerBDot::ControllerBDot()
    : // Initialize Members 
    m_min(Param::Actuators::m_min),
    m_max(Param::Actuators::m_max),
    K_Bdot(Param::Controller::K_Bdot),
    B_prev(Vector3::Zero()),
    B_dot(Vector3::Zero()),
    alpha_Bdot(Param::Controller::alpha_BDot),
    Bdot_num_filt(Vector3::Zero()),
    beta_fuse(Param::Controller::beta_fuse)
{
    // Constructor body (if needed)
}

// Methods
ControllerBDot::Vector3 ControllerBDot::update(const Measurements& measurements, const Param::Vector11& states_hat, Scalar dt)
{
    // Extract magnetic field measurement
    Vector3 B_now = measurements.segment<3>(6); // Assuming B field is in elements 6-8
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
    states_m(0) = states_hat(0); // q0
    states_m(1) = states_hat(1); // q1
    states_m(2) = states_hat(2); // q2
    states_m(3) = states_hat(3); // q3
    states_m(4) = states_hat(4); // omega_x
    states_m(5) = states_hat(5); // omega_y
    states_m(6) = states_hat(6); // omega_z

    if (B_prev.isZero()) {
        B_prev = B_now;
        B_dot = Vector3::Zero();
        return Vector3::Zero();
    }
    // Apply low-pass filter to B_dot
    Bdot_num_filt = (static_cast<Scalar>(1) - alpha_Bdot)*(B_now - B_prev) / dt + alpha_Bdot * Bdot_num_filt;
    B_dot = (static_cast<Scalar>(1)-beta_fuse)*(-omega_meas.cross(B_now)) + beta_fuse*Bdot_num_filt;
    m_tilde = -K_Bdot*B_dot;
    // Saturate magnetic moment
    if (B_now.norm() > static_cast<Scalar>(1e-9)) {
        Bh = B_now / B_now.norm();

        // 1. Explicitly calculate the outer product into a Matrix3
        Param::Matrix3 outerProduct = Math::outerProduct(Bh, Bh);

        // 2. Perform the subtraction from Identity
        P_perp = Param::Matrix3::Identity() - outerProduct;

        m_tilde = P_perp * m_tilde;
    }
    
    m_sat = saturateSymmetric(m_tilde, m_max);
    // Update B_prev
    B_prev = B_now;
    return m_sat;
}