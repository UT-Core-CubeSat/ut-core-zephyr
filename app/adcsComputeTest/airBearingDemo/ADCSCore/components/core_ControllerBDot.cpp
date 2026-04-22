#include "core_ControllerBDot.hpp"
#include "core_Saturate.hpp"
#include <cmath>
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

}

// Methods
ControllerBDot::Vector3 ControllerBDot::update(const Measurements& measurements, const Param::Vector11& states_hat, Scalar dt)
{
    // Extract magnetic field measurement
    Vector3 B_now = measurements.segment<3>(6); // Assuming B field is in elements 6-8
    Vector3 omega_meas = measurements.segment<3>(3); // Assuming angular velocity is in elements 3-5
    Vector3 m_tilde;
    Vector3 Bh;
    Vector3 m_sat;
    (void)states_hat;

    if (B_prev.isZero()) {
        B_prev = B_now;
        B_dot = Vector3::Zero();
        return Vector3::Zero();
    }

    if (!std::isfinite(dt) || dt <= static_cast<Scalar>(1e-6)) {
        return Vector3::Zero();
    }

    // RTOS Optimization: Pre-compute norm once and avoid redundant computations
    Scalar B_norm = B_now.norm();
    Scalar B_norm_threshold = static_cast<Scalar>(1e-9);
    
    // Apply low-pass filter to B_dot
    Scalar dt_inv = static_cast<Scalar>(1.0) / dt;  // Pre-compute reciprocal (faster than division)
    Bdot_num_filt = (static_cast<Scalar>(1) - alpha_Bdot) * (B_now - B_prev) * dt_inv + alpha_Bdot * Bdot_num_filt;
    B_dot = (static_cast<Scalar>(1) - beta_fuse) * (-omega_meas.cross(B_now)) + beta_fuse * Bdot_num_filt;
    m_tilde = -K_Bdot * B_dot;
    
    // Saturate magnetic moment and project perpendicular to B-field (rank-1 projection)
    if (B_norm > B_norm_threshold) {
        // Use pre-computed norm to avoid redundant calculation
        Scalar inv_norm = static_cast<Scalar>(1.0) / B_norm;
        Bh = B_now * inv_norm;

        // Direct rank-1 projection: m_tilde -= Bh * (Bh.dot(m_tilde))
        // This removes the parallel component; no wasted torque along B-field
        m_tilde -= Bh * Bh.dot(m_tilde);
    }
    
    m_sat = saturateSymmetric(m_tilde, m_max);
    // Four long-face panel coils provide no body-Z magnetic authority in this test setup.
    m_sat(2) = static_cast<Scalar>(0.0);
    // Update B_prev
    B_prev = B_now;
    return m_sat;
}