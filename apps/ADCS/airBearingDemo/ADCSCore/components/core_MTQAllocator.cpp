#include "core_MTQAllocator.hpp"

MTQAllocator::AllocOutput MTQAllocator::allocate(const Vector3& m_body, const Vector3& B_measured) const
{
    AllocOutput out;
    
    // Body dipole components
    Real m_x = m_body(0);  // X-axis dipole [A·m²]
    Real m_y = m_body(1);  // Y-axis dipole [A·m²]
    // m_z is clamped to 0 by B-dot controller, ignored here
    
    // Coil constants (K_coil = N_turns * A_coil)
    // RTOS Optimization: Cache coil constants locally to avoid repeated parameter lookups
    const Real K_x = Param::Actuators::Coils::K_coil_x;
    const Real K_nx = Param::Actuators::Coils::K_coil_nx;
    const Real K_y = Param::Actuators::Coils::K_coil_y;
    const Real K_ny = Param::Actuators::Coils::K_coil_ny;
    const Real I_max = Param::Actuators::Coils::I_max;
    
    // Pre-compute reciprocals to avoid division in hot path (multiply is faster than divide)
    const Real K_x_inv = (std::abs(K_x) > static_cast<Real>(1e-9)) ? static_cast<Real>(1.0) / K_x : static_cast<Real>(0.0);
    const Real K_nx_inv = (std::abs(K_nx) > static_cast<Real>(1e-9)) ? static_cast<Real>(1.0) / K_nx : static_cast<Real>(0.0);
    const Real K_y_inv = (std::abs(K_y) > static_cast<Real>(1e-9)) ? static_cast<Real>(1.0) / K_y : static_cast<Real>(0.0);
    const Real K_ny_inv = (std::abs(K_ny) > static_cast<Real>(1e-9)) ? static_cast<Real>(1.0) / K_ny : static_cast<Real>(0.0);
    
    // Allocate body dipole to per-face currents using pre-computed reciprocals
    // I_face = m_cmd * K_coil_inv (multiply is ~3-5x faster than divide on embedded MCU)
    Real I_Xpos = m_x * K_x_inv;
    Real I_Xneg = -m_x * K_nx_inv;  // Negative to oppose
    Real I_Ypos = m_y * K_y_inv;
    Real I_Yneg = -m_y * K_ny_inv;  // Negative to oppose
    
    // Apply current saturation per face (inline saturate for speed)
    I_Xpos = (I_Xpos > I_max) ? I_max : ((I_Xpos < -I_max) ? -I_max : I_Xpos);
    I_Xneg = (I_Xneg > I_max) ? I_max : ((I_Xneg < -I_max) ? -I_max : I_Xneg);
    I_Ypos = (I_Ypos > I_max) ? I_max : ((I_Ypos < -I_max) ? -I_max : I_Ypos);
    I_Yneg = (I_Yneg > I_max) ? I_max : ((I_Yneg < -I_max) ? -I_max : I_Yneg);

    // Pack per-face currents: [I_Xpos, I_Xneg, I_Ypos, I_Yneg]
    out.face_current(0) = I_Xpos;
    out.face_current(1) = I_Xneg;
    out.face_current(2) = I_Ypos;
    out.face_current(3) = I_Yneg;
    
    // Compute reference field for each face
    // Simplified model: B_face ≈ (μ₀ / 4π) * (2 * dipole_moment) / r³
    // For a calibrated setup with known coil geometry:
    // B_ref = C_B * I where C_B depends on coil geometry
    // 
    // Placeholder: Assume each coil produces ~1 µT/A as a nominal calibration
    // (User calibrates with Helmholtz cage sweep and updates this)
    constexpr Real B_per_amp = static_cast<Real>(1e-5);  // [T/A] placeholder
    
    // Use absolute values for field magnitude (always positive)
    out.face_b_ref(0) = (I_Xpos >= 0) ? I_Xpos * B_per_amp : -I_Xpos * B_per_amp;
    out.face_b_ref(1) = (I_Xneg >= 0) ? I_Xneg * B_per_amp : -I_Xneg * B_per_amp;
    out.face_b_ref(2) = (I_Ypos >= 0) ? I_Ypos * B_per_amp : -I_Ypos * B_per_amp;
    out.face_b_ref(3) = (I_Yneg >= 0) ? I_Yneg * B_per_amp : -I_Yneg * B_per_amp;
    
    (void)B_measured;  // Reserved for future adaptive field estimation
    
    return out;
}

Real MTQAllocator::compute_coil_field_estimate(Real coil_current) const
{
    // Placeholder: linear relationship (calibrate with actual coil)
    constexpr Real B_per_amp = static_cast<Real>(1e-5);  // [T/A]
    return std::abs(coil_current) * B_per_amp;
}
