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
    beta_fuse(Param::Controller::beta_fuse),
    coil_state(Vector4::Zero()),
    m_bang_state(Vector3::Zero())
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

    const Scalar thresh_on = static_cast<Scalar>(0.10) * m_max;
    const Scalar thresh_off = static_cast<Scalar>(0.03) * m_max; 

    const Scalar omega_norm = omega_meas.norm();
    const Scalar omega_gate = static_cast<Scalar>(0.5) * Param::deg2rad; // ~0.5 deg/s
    
    if (omega_norm < omega_gate) {
        m_bang_state = Vector3::Zero();
        coil_currents_out = Vector4::Zero();
        coil_state = Vector4::Zero();
        B_prev = B_now;
        return m_bang_state;
    }
    for (int i = 0; i < 3; ++i) {
        Scalar cmd = m_tilde(i);
        Scalar current = m_bang_state(i);

        if (current > 0) {
            m_bang_state(i) = (cmd > thresh_off) ? m_max : static_cast<Scalar>(0);
        } else if (current < 0) {
            m_bang_state(i) = (cmd < -thresh_off) ? -m_max : static_cast<Scalar>(0);
        } else {
            if      (cmd >  thresh_on) m_bang_state(i) =  m_max;
            else if (cmd < -thresh_on) m_bang_state(i) = -m_max;
        }
    }
    
    // Four long-face panel coils provide no body-Z magnetic authority in this test setup.
    // Update B_prev
    B_prev = B_now;
    // Update coil state (simplified hysteresis logic)
    coil_state = dipoleToCoilCurrents(m_bang_state);
    coil_currents_out = coil_state;
    return m_bang_state;
}

ControllerBDot::Vector4 ControllerBDot::dipoleToCoilCurrents(const Vector3& m_cmd)
{
    // m_cmd is already bang-bang (±m_max or 0), so just decompose by sign.
    // Layout: [Xpos_en, Xneg_en, Ypos_en, Yneg_en]
    Scalar signals[4] = {
         m_cmd(0),
        -m_cmd(0),
         m_cmd(1),
        -m_cmd(1)
    };

    for (int i = 0; i < 4; ++i) {
        coil_state(i) = (signals[i] > static_cast<Scalar>(0))
                        ? static_cast<Scalar>(1)
                        : static_cast<Scalar>(0);
    }
    return coil_state;
}