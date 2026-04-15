#ifndef TWO_BODY_HPP
#define TWO_BODY_HPP

#include "Parameters.hpp"

namespace TwoBody {
    Param::Vector3 two_body(const Param::Real& mu_E, Param::Real& r_E, Param::Vector3& R);
} // namespace TwoBody

#endif // TWO_BODY_HPP