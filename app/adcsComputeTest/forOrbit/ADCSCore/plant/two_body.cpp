#include "two_body.hpp"
#include <cmath>

namespace TwoBody {

PlantParam::Vector3 two_body(const PlantParam::Real& mu_E, PlantParam::Real& r_E, PlantParam::Vector3& R) {
    // Unpack position and calculate norm
    PlantParam::Real x = R(0);
    PlantParam::Real y = R(1);
    PlantParam::Real z = R(2);
    PlantParam::Real r = R.norm();

    // Prevent division by zero if r is near 0
    if (r < static_cast<PlantParam::Real>(1e-6)) {
        return PlantParam::Vector3::Zero();
    }

    // 1. Model Newton's 2-body problem (Point Mass)
    // a_g = -mu_E * R / r^3
    PlantParam::Vector3 a_g = -mu_E * R / (r * r * r);

    // 2. Model J2 perturbation (Earth's oblateness)
    // Constants from Param namespace
    PlantParam::Real J2 = PlantParam::Earth::J2;
    
    // Intermediate terms to match MATLAB logic: -(3/2)*J2*(mu_E/r^2)*(r_E/r)^2
    PlantParam::Real r_ratio = r_E / r;
    PlantParam::Real common_factor = static_cast<PlantParam::Real>(-1.5) * J2 * (mu_E / (r * r)) * (r_ratio * r_ratio);

    // Geometry terms
    PlantParam::Real z_over_r_sq = (z / r) * (z / r);
    
    PlantParam::Vector3 a_J2;
    a_J2(0) = (static_cast<PlantParam::Real>(1.0) - static_cast<PlantParam::Real>(5.0) * z_over_r_sq) * (x / r); // x-component
    a_J2(1) = (static_cast<PlantParam::Real>(1.0) - static_cast<PlantParam::Real>(5.0) * z_over_r_sq) * (y / r); // y-component
    a_J2(2) = (static_cast<PlantParam::Real>(3.0) - static_cast<PlantParam::Real>(5.0) * z_over_r_sq) * (z / r); // z-component
    
    a_J2 *= common_factor;

    // Total acceleration
    return a_g + a_J2;
}

} // namespace TwoBody