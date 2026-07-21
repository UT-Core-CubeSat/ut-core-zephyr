#ifndef CORE_MTQ_ALLOCATOR_HPP
#define CORE_MTQ_ALLOCATOR_HPP

#include "core_Parameters.hpp"
#include "core_Saturate.hpp"
/**
 * @file core_MTQAllocator.hpp
 * @brief Per-face magnetorquer current and reference field allocator
 * 
 * Converts body-frame dipole command [A·m²] to per-face coil currents [A]
 * and reference field magnitudes [T] for CAN transmission.
 * 
 * Assumes 4-face geometry: +X, -X, +Y, -Y (Z-axis inactive)
 */
class MTQAllocator {
public:
    using Vector3 = Param::Vector3;
    using Vector4 = Param::Vector4;
    using Real = Param::Real;

    MTQAllocator() = default;

    /**
     * @brief Allocate body dipole command to per-face coil currents.
     * 
     * @param m_body Body-frame dipole command [A·m²]: (m_x, m_y, m_z)
     *               Note: m_z should already be clamped to 0 by B-dot controller
     * @param B_measured Measured magnetic field [T] in body frame (used for reference field calc)
     * 
     * @return struct with:
     *   - face_current[4]: Per-face coil current [A] in order [I_Xpos, I_Xneg, I_Ypos, I_Yneg]
     *   - face_b_ref[4]:   Reference field magnitude [T] each face will produce
     */
    struct AllocOutput {
        Vector4 face_current;
        Vector4 face_b_ref;
    };

    AllocOutput allocate(const Vector3& m_body, const Vector3& B_measured) const;

private:
    /**
     * @brief Estimate reference field magnitude for a given coil current.
     * 
     * Simplified dipole approximation: B_coil ≈ (μ₀ * N * I) / (2*R³)
     * where μ₀ = 4π × 10⁻⁷, N = turns, I = current, R ≈ coil_radius
     * 
     * For a calibrated coil: B_ref = C_field * I where C_field is pre-computed.
     */
    Real compute_coil_field_estimate(Real coil_current) const;
};

#endif // CORE_MTQ_ALLOCATOR_HPP
