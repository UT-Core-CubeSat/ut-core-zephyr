#ifndef TWO_BODY_HPP
#define TWO_BODY_HPP

#include "Plant_Parameters.hpp"

namespace TwoBody {
    PlantParam::Vector3 two_body(const PlantParam::Real& mu_E, PlantParam::Real& r_E, PlantParam::Vector3& R);
} // namespace TwoBody

#endif // TWO_BODY_HPP