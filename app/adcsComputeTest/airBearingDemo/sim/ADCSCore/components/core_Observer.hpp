#ifndef CORE_OBSERVER_HPP
#define CORE_OBSERVER_HPP

#include "core_Parameters.hpp"
#include "core_HelperFunctions.hpp"

class ObserverClass {
public: 
    using StateVector = Param::Vector11;
    using Scalar = Param::Real;
    using TimeReal = Param::TimeReal;
    using Vector3 = Param::Vector3;
    using Vector4 = Param::Vector4;
    using Quat = Param::Vector4;

    ObserverClass();
    StateVector update(const Param::Vector13& measurements, const TimeReal& t, const Scalar& dt);
    
private: 
    HelperFunctions helpers;

    void propagate(const Vector3& omega_meas, Scalar dt);
    void updateWithGravity(const Vector3& accel_meas, Scalar dt);  // NEW
    void updateWithMagnetometer(const Vector3& mag_meas, Scalar dt);  // NEW
    Param::Matrix3 skew(const Vector3& v);
    Quat quatMultiply(const Quat& q1, const Quat& q2);

    Quat last_q_star;
    Quat q_hat;
    Vector3 beta_hat;
    Vector3 g_ref;  // NEW: Reference gravity direction in inertial frame
    Vector3 B_ref;  // NEW: Reference magnetic field direction in inertial frame
    Param::Matrix6 P;
    Param::Matrix6 G;
    Param::Matrix6 Q;
    Vector3 sigma_v, sigma_u;
    Param::TimeReal epoch_time;
    Param::TimeReal current_time;
};

#endif