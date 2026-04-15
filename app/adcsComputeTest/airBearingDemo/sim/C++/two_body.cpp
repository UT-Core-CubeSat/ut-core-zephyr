#include "two_body.hpp"
#include <cmath>

namespace TwoBody {

Param::Vector3 two_body(const Param::Real& mu_E, Param::Real& r_E, Param::Vector3& R) {
    // Unpack position and calculate norm
    Param::Real x = R(0);
    Param::Real y = R(1);
    Param::Real z = R(2);
    Param::Real r = R.norm();

    // Prevent division by zero if r is near 0
    if (r < 1e-6) {
        return Param::Vector3::Zero();
    }

    // 1. Model Newton's 2-body problem (Point Mass)
    // a_g = -mu_E * R / r^3
    Param::Vector3 a_g = -mu_E * R / (r * r * r);

    // 2. Model J2 perturbation (Earth's oblateness)
    // Constants from Param namespace
    Param::Real J2 = Param::Earth::J2;
    
    // Intermediate terms to match MATLAB logic: -(3/2)*J2*(mu_E/r^2)*(r_E/r)^2
    Param::Real r_ratio = r_E / r;
    Param::Real common_factor = -1.5 * J2 * (mu_E / (r * r)) * (r_ratio * r_ratio);

    // Geometry terms
    Param::Real z_over_r_sq = (z / r) * (z / r);
    
    Param::Vector3 a_J2;
    a_J2(0) = (1.0 - 5.0 * z_over_r_sq) * (x / r); // x-component
    a_J2(1) = (1.0 - 5.0 * z_over_r_sq) * (y / r); // y-component
    a_J2(2) = (3.0 - 5.0 * z_over_r_sq) * (z / r); // z-component
    
    a_J2 *= common_factor;

    // Total acceleration
    return a_g + a_J2;
}

} // namespace TwoBody