#ifndef CORE_OBSERVER_HPP
#define CORE_OBSERVER_HPP

#include "core_Parameters.hpp"
#include "core_HelperFunctions.hpp"

class ObserverClass {
public: 

    // Type Aliases for readability 
    using StateVector = Param::Vector17;
    using Reference = Param::Vector10;
    using Scalar = Param::Real;
    using Vector3 = Param::Vector3;
    using Vector4 = Param::Vector4;
    using Quat = Param::Vector4;

    // Constructor 
    ObserverClass();

    StateVector update(const Param::Vector29& measurements, const Scalar& t);
    
private: 
    // Initialization Helpers
    HelperFunctions helpers;

    // Private methods 
    void propagate(const Vector3& omega_meas);
    void star_tracker_update(const Quat& q_meas);
    void quest_update(const Quat& q_meas);
    Quat quest_algorithm(const Scalar& JD, 
                         const Vector3& r_eci, 
                         const Vector3& b_sun, 
                         const Vector3& b_mag);

    // Observer Helpers 
    Param::Matrix3 skew(const Vector3& v);
    Quat quatMultiply(const Quat& q1, const Quat& q2);
    Vector3 compute_r_mag(const Scalar& JD, 
                             const Vector3& r_eci);
    Vector3 synthesize_sun_vector(const Param::Vector6& y_css, 
                                  const Scalar& I_max);

    // Private Members 
    Quat last_q_star;
    Quat q_hat;
    Vector3 beta_hat;
    Param::Matrix6 P;
    Param::Matrix6 G;
    Param::Matrix6 Q;
    Vector3 sigma_v, sigma_u;
    Param::Matrix3 R_star;
    Param::Matrix3 R_quest;
    Scalar Ts;
    Scalar epoch_time;      // Unix timestamp at simulation start
    Scalar current_time;    // Current Unix timestamp (epoch + t)
    Vector3 omega_earth;
    bool use_star_tracker;
    bool use_magnetometer;
    bool use_sun_sensor;
    Scalar last_star_update_time;
    Scalar last_quest_update_time;  // Rate limit QUEST updates
    Scalar T_star;
    Scalar T_quest;  // QUEST update period (same as star tracker)
    Scalar I_max;
    Scalar I_threshold;
};

#endif // CORE_OBSERVER_HPP