#ifndef SENSORS_HPP
#define SENSORS_HPP

#include "Plant_Parameters.hpp"
#include "../components/core_HelperFunctions.hpp"
#include <random>

class SensorsClass {
public: 

    // Type Aliases for readability 
    using StateVector = Param::Vector11;
    using Reference = Param::Vector10;
    using Measurements = Param::Vector13;
    using Scalar = Param::Real;
    using TimeReal = Param::TimeReal;
    using Vector3 = Param::Vector3;
    using Vector4 = Param::Vector4;
    using Quat = Param::Vector4;

    // Constructor 
    SensorsClass();

    Measurements measurements(const StateVector& states_dot, 
                              const StateVector& states, 
                              const Scalar& t);

private: 
    // Private Methods 
    void updateBiasStates(const Scalar& dt);
    Vector3 accelerometer(const StateVector& states_dot, 
                          const StateVector& states);
    Vector3 gyroscope(const StateVector& states);
    Param::Vector3 magnetometer(const StateVector& states);
    Vector4 reaction_wheels(const StateVector& states);
    
    // Static RNG - seeded once at program start to match MATLAB's rng(42)
    static std::mt19937& get_rng();
    
    template<int N>
    Math::Vec<N> randn();

    // Private Members 
    // time tracking

    HelperFunctions helpers;
    TimeReal epoch_time;      // Unix timestamp at simulation start
    TimeReal current_time;    // Current Unix timestamp (epoch + t)
    Scalar Ts;
    // accelerometer
    Vector3 beta_a;
    Vector3 beta_a_state;
    Vector3 sigma_a;
    Vector3 sigma_bias_walk_a;

    // gyro
    Vector3 sigma_gyro;
    Vector3 beta_gyro;
    Vector3 beta_gyro_state;
    Vector3 sigma_bias_walk_gyro;

    // sun sensor
    Scalar I_max;
    Param::Vector6 beta_css;
    Param::Vector6 sigma_css;

    // magnetometer
    Vector3 beta_mag;
    Vector3 beta_mag_state;
    Vector3 sigma_mag;
    Vector3 sigma_bias_walk_mag;
    Vector3 magnitude;

    // Wheels
    Vector4 omega_w_previous;
    Vector4 omega_w_meas;
    Scalar sigma_w;
    Scalar alpha_w;
};

#endif // SENSORS_HPP