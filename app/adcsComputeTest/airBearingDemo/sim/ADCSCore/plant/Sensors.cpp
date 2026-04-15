#include "Sensors.hpp"
#include <iostream> // For debugging purposes
#include <random>

// Constructor 
SensorsClass::SensorsClass()
    : // Initialize Members
    epoch_time(PlantParam::SimTime::epoch_timestamp),
    current_time(PlantParam::SimTime::epoch_timestamp),
    Ts(PlantParam::SimTime::Ts),
    // accelerometer
    beta_a(PlantParam::Sensors::beta_a),
    sigma_a(PlantParam::Sensors::sigma_a),
    // gyro
    sigma_gyro(PlantParam::Sensors::sigma_gyro),
    beta_gyro(PlantParam::Sensors::beta_gyro),
    // magnetometer
    beta_mag(PlantParam::Sensors::beta_mag),
    sigma_mag(PlantParam::Sensors::sigma_mag),
    magnitude(PlantParam::Apparatus::helmholtz_mag),
    // reaction wheels
    omega_w_previous(PlantParam::InitialState.omega_wheel),
    omega_w_meas(omega_w_previous),
    sigma_w(PlantParam::Sensors::sigma_wheel),
    alpha_w(PlantParam::Sensors::alpha_wheel)
{
    
}

SensorsClass::Measurements SensorsClass::measurements(const StateVector& states_dot, 
                                        const StateVector& states, 
                                        const Scalar& t) {
    // Compute current Unix timestamp from epoch + simulation time
    current_time = epoch_time + t;

    // update accelerometer
    Vector3 y_a = accelerometer(states_dot, states);

    // update gyro 
    Vector3 y_gyro = gyroscope(states);
    
    // update magnetometer 
    Vector3 y_B = magnetometer(states);

    // update wheels 
    Vector4 y_w = reaction_wheels(states);

    // pack measurements vector 
    Measurements meas;
    meas.setSegment(0, y_a);
    meas.setSegment(3, y_gyro);
    meas.setSegment(6, y_B);
    meas.setSegment(9, y_w);
    return meas;
}

SensorsClass::Vector3 SensorsClass::accelerometer(const StateVector& states_dot, 
                                                   const StateVector& states) {
    // For air bearing: accelerometer measures gravity in body frame
    // (no translational motion, so specific force = -g_body)
    
    // Get quaternion from states
    Quat q = states.segment<4>(0);
    
    // Gravity in inertial frame [0, 0, -g]
    Vector3 g_inertial = Vector3{static_cast<Scalar>(0.0), 
                                  static_cast<Scalar>(0.0), 
                                  static_cast<Scalar>(-9.80665)};
    
    // Rotate to body frame: g_b = q* ⊗ g_i ⊗ q
    Quat q_conj;
    q_conj(0) = q(0);
    q_conj.setSegment(1, -q.segment<3>(1));
    Vector3 g_body = helpers.quatRotate(q_conj, g_inertial);
    
    // Accelerometer measures specific force = acceleration - gravity
    // On air bearing with no translation: a = 0, so accel_meas = -g_body = +g_up
    Vector3 a_sensed = -g_body;  // Points "up" in body frame
    
    // Add bias and noise
    Vector3 y_a = a_sensed + beta_a + sigma_a.cwiseProduct(randn<3>());

    return y_a;
}

SensorsClass::Vector3 SensorsClass::gyroscope(const StateVector& states) {
    // model gyroscope 
    Vector3 omega = states.segment<3>(4);

    // add bias and noise 
    Vector3 y_gyro = omega + beta_gyro + sigma_gyro.cwiseProduct(randn<3>());

    return y_gyro;
}

SensorsClass::Vector3 SensorsClass::magnetometer(const StateVector& states) {
    // Get quaternion from states
    Quat q = states.segment<4>(0);
    
    // Helmholtz field is fixed in lab/inertial frame
    // Rotate to body frame: B_body = q* ⊗ B_inertial ⊗ q
    Quat q_conj;
    q_conj(0) = q(0);
    q_conj.setSegment(1, -q.segment<3>(1));
    Vector3 B_body = helpers.quatRotate(q_conj, magnitude);
    
    // Add bias and noise
    Vector3 y_B = B_body + beta_mag + sigma_mag.cwiseProduct(randn<3>());
    return y_B;
}

SensorsClass::Vector4 SensorsClass::reaction_wheels(const StateVector& states) {
    // 1. Extract True Wheel Speeds (Indices 7-10)
    Vector4 omega_w = states.segment<4>(7);

    // 2. Generate Gaussian Noise
    Vector4 noise = randn<4>() * sigma_w;
    
    // 3. Raw Measurement
    Vector4 meas_raw = omega_w + noise;

    // 4. Low Pass Filter (IIR)
    omega_w_meas = meas_raw * alpha_w + omega_w_meas * (static_cast<Scalar>(1.0) - alpha_w);

    return omega_w_meas;
}

// Global RNG - must be seeded at program start to match MATLAB's rng(42) timing
std::mt19937& SensorsClass::get_rng() {
    static std::mt19937 gen(42);  // Seeded once, matching MATLAB
    return gen;
}

template<int N> 
Math::Vec<N> SensorsClass::randn() {
    static std::normal_distribution<Scalar> dist(static_cast<Scalar>(0.0), static_cast<Scalar>(1.0));
    auto& gen = get_rng();

    Math::Vec<N> vec;
    for (int i = 0; i < N; ++i) {
        vec(i) = dist(gen);
    }
    return vec;
}

// Explicit template instantiations
template Math::Vec<1> SensorsClass::randn<1>();
template Math::Vec<3> SensorsClass::randn<3>();
template Math::Vec<4> SensorsClass::randn<4>();
template Math::Vec<6> SensorsClass::randn<6>();
