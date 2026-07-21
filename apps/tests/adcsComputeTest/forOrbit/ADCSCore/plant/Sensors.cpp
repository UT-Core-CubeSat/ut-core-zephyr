#include "Sensors.hpp"
#include <iostream> // For debugging purposes
#include "two_body.hpp"
#include <random>

// Constructor 
SensorsClass::SensorsClass()
    : // Initialize Members
    epoch_time(PlantParam::SimTime::epoch_timestamp),
    current_time(PlantParam::SimTime::epoch_timestamp),
    Ts(PlantParam::SimTime::Ts),
    mu_E(PlantParam::Earth::mu_E),
    r_E(PlantParam::Earth::r_E),
    // accelerometer
    beta_a(PlantParam::Sensors::beta_a),
    sigma_a(PlantParam::Sensors::sigma_a),
    // gyro
    sigma_gyro(PlantParam::Sensors::sigma_gyro),
    beta_gyro(PlantParam::Sensors::beta_gyro),
    // star tracker
    star_update(PlantParam::Sensors::T_star),
    y_star_previous(PlantParam::Orbit::InitialState.q0),
    sigma_star(PlantParam::Sensors::sigma_star),
    beta_star(PlantParam::Sensors::beta_star),
    small_angle_tol(PlantParam::Sensors::small_angle_tol),
    T_star(PlantParam::Sensors::T_star),
    previous_star_update(static_cast<Scalar>(0.0)),
    // sun sensor
    I_max(PlantParam::Sensors::I_max),
    beta_css(PlantParam::Sensors::beta_css),
    sigma_css(PlantParam::Sensors::sigma_css),
    // magnetometer
    beta_mag(PlantParam::Sensors::beta_mag),
    sigma_mag(PlantParam::Sensors::sigma_mag),
    // GPS
    omega_earth(PlantParam::Earth::omega_earth),
    K_gps(PlantParam::Sensors::K_gps),
    T_gps(PlantParam::Sensors::T_gps),
    nu(PlantParam::Vector3::Zero()),
    sigma_cep(PlantParam::Sensors::sigma_gps_cep),
    sigma_V(PlantParam::Sensors::sigma_V),
    y_gps_previous(PlantParam::Vector6::Zero()),
    previous_gps_update(static_cast<Scalar>(0.0)),
    // reaction wheels
    omega_w_previous(PlantParam::Orbit::InitialState.omega_wheel),
    omega_w_meas(omega_w_previous),
    sigma_w(PlantParam::Sensors::sigma_wheel),
    alpha_w(PlantParam::Sensors::alpha_wheel)
{
    // Initialize GPS in ECEF frame (not ECI) to match gps() output
    TimeReal jd_init = helpers.julianDate(epoch_time);
    PlantParam::Matrix3 C_eci2ecef_init = helpers.dcmeci2ecef(jd_init);
    Vector3 r_ecef_init = C_eci2ecef_init * PlantParam::Orbit::InitialState.r_ECI;
    Vector3 v_ecef_init = C_eci2ecef_init * PlantParam::Orbit::InitialState.v_ECI - 
                          omega_earth.cross(r_ecef_init);
    y_gps_previous.setSegment(0, r_ecef_init);
    y_gps_previous.setSegment(3, v_ecef_init);
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

    // check if star tracker can update 
    Quat y_star;
    if (t - previous_star_update >= T_star) {
        y_star = star_tracker(states);
        y_star_previous = y_star;
        previous_star_update = t;
    } else {
        y_star = y_star_previous;
    }

    // update sun sensor 
    PlantParam::Vector6 y_css = sun_sensor(states);
    
    // update magnetometer 
    Vector3 y_B = magnetometer(states);

    // check if gps can update 
    PlantParam::Vector6 y_gps;
    if (t - previous_gps_update >= T_gps) {
        y_gps = gps(states);
        y_gps_previous = y_gps;
        previous_gps_update = t;
    } else {
        y_gps = y_gps_previous;
    }

    // update wheels 
    Vector4 y_w = reaction_wheels(states);

    // pack measurements vector 
    Measurements meas;
    meas.setSegment(0, y_a);
    meas.setSegment(3, y_gyro);
    meas.setSegment(6, y_star);
    meas.setSegment(10, y_css);
    meas.setSegment(16, y_B);
    meas.setSegment(19, y_gps);
    meas.setSegment(25, y_w);
    return meas;
}

SensorsClass::StateVector SensorsClass::measurements2states(const Measurements& measurements) {
    // unpack measurements 
    Vector3 y_a = measurements.segment<3>(0);
    Vector3 y_gyro = measurements.segment<3>(3);
    Quat y_star = measurements.segment<4>(6);
    PlantParam::Vector6 y_css = measurements.segment<6>(10);
    Vector3 y_B = measurements.segment<3>(16);
    PlantParam::Vector6 y_gps = measurements.segment<6>(19);

    PlantParam::Matrix3 C_eci2ecef = helpers.dcmeci2ecef(helpers.julianDate(current_time));

    // find R_hat in ECI frame 
    Vector3 gps_pos = y_gps.segment<3>(0);
    Vector3 R_hat = C_eci2ecef.transpose() * gps_pos;

    // find V_hat in ECI frame 
    Vector3 gps_vel = y_gps.segment<3>(3);
    Vector3 V_hat = C_eci2ecef.transpose() * gps_vel + omega_earth.cross(R_hat);

    // find q_hat
    Quat q_hat = y_star;
    q_hat.normalize();

    // find omega_hat in body frame 
    Vector3 omega_hat = y_gyro;

    // pack states_hat vector 
    StateVector states_hat;
    states_hat.setSegment(0, R_hat);
    states_hat.setSegment(3, V_hat);
    states_hat.setSegment(6, q_hat);
    states_hat.setSegment(10, omega_hat);
    return states_hat;
}

SensorsClass::Vector3 SensorsClass::accelerometer(const StateVector& states_dot, 
                              const StateVector& states) {
    // model accelerometer   
    Vector3 R = states.segment<3>(0);
    Quat q = states.segment<4>(6);
    Vector3 a_i = states_dot.segment<3>(3);
    
    // calculate acceleration due to gravity in ECI frame
    Vector3 g_i = TwoBody::two_body(mu_E, r_E, R);

    // calculate the acceleration the sensors will be able to measure
    Vector3 a_mi = a_i - g_i;

    // passive rotation into body frame (assuming quaternion scalar first)
    Quat q_star = helpers.quatconj(q);
    Vector3 a_b = helpers.quatRotate(q_star, a_mi);

    // add bias and noise 
    Vector3 y_a = a_b + beta_a + sigma_a.cwiseProduct(randn<3>());

    return y_a;
}

SensorsClass::Vector3 SensorsClass::gyroscope(const StateVector& states) {
    // model gyroscope 
    Vector3 omega = states.segment<3>(10);

    // add bias and noise 
    Vector3 y_gyro = omega + beta_gyro + sigma_gyro.cwiseProduct(randn<3>());

    return y_gyro;
}

SensorsClass::Quat SensorsClass::star_tracker(const StateVector& states) {
    // model star tracker 
    Quat q_true = states.segment<4>(6);
    Vector3 q_noise_vec;
    Scalar q_noise_scl;
    
    // determine if startracker will update
    // Use same seeded RNG as randn() by transforming normal to uniform via CDF
    Scalar normal_sample = randn<1>()(0);
    Scalar random_val = static_cast<Scalar>(0.5) * (static_cast<Scalar>(1.0) + std::erf(normal_sample / std::sqrt(static_cast<Scalar>(2.0))));
    
    if (star_update < random_val) {
        return y_star_previous; // no update, return previous measurement
    } else {    
        if (sigma_star.norm() < small_angle_tol) {
            q_noise_vec = sigma_star / static_cast<Scalar>(2.0);
            q_noise_scl = static_cast<Scalar>(1.0);
        } else {
            // no small angle approximation 
            Scalar angle = sigma_star.norm();
            Vector3 n_hat = sigma_star / angle;
            q_noise_vec = n_hat * std::sin(angle / static_cast<Scalar>(2.0));
            q_noise_scl = std::cos(angle / static_cast<Scalar>(2.0));
        }
        Quat q_eta;
        q_eta(0) = q_noise_scl;
        q_eta(1) = q_noise_vec(0);
        q_eta(2) = q_noise_vec(1);
        q_eta(3) = q_noise_vec(2);

        // bias is fixed so no conditional needed 
        // bias quaternion - convert bias angles to quaternion 
        Vector3 q_bias_vec;
        Scalar q_bias_scl;
        Quat q_beta;
        if (beta_star.norm() < small_angle_tol) {
            q_bias_vec = beta_star / static_cast<Scalar>(2.0);
            q_bias_scl = static_cast<Scalar>(1.0);
            q_beta(0) = q_bias_scl;
            q_beta(1) = q_bias_vec(0);
            q_beta(2) = q_bias_vec(1);
            q_beta(3) = q_bias_vec(2);
        } else {
            // no small angle approximation 
            Scalar angle_b = beta_star.norm();
            Vector3 n_hat_b = beta_star / angle_b;
            q_bias_vec = n_hat_b * std::sin(angle_b / static_cast<Scalar>(2.0));
            q_bias_scl = std::cos(angle_b / static_cast<Scalar>(2.0));
            q_beta(0) = q_bias_scl;
            q_beta(1) = q_bias_vec(0);
            q_beta(2) = q_bias_vec(1);
            q_beta(3) = q_bias_vec(2);
        }

        Quat y_star = helpers.quatMultiply(q_eta, helpers.quatMultiply(q_beta, q_true));
        y_star.normalize();
        return y_star;
    }
}

PlantParam::Vector6 SensorsClass::sun_sensor(const StateVector& states) {
    // model sun sensor 
    Quat q = states.segment<4>(6);
    Vector3 R_sat = states.segment<3>(0);

    // get current time and convert to juliandate
    TimeReal jd = helpers.julianDate(current_time);

    // calculate position of sun relative to earth
    Vector3 R_sun = helpers.earth2sun(jd).normalized();

    // calculate position vector from satellite to sun 
    Vector3 S_vec = R_sun - R_sat;
    Vector3 s_i_hat = S_vec.normalized();

    // rotate sun vector into body frame 
    Quat q_star = helpers.quatconj(q);
    Vector3 s_b_hat = helpers.quatRotate(q_star, s_i_hat);

    Vector3 I_p = s_b_hat.cwiseMax(static_cast<Scalar>(0.0)) * I_max;
    Vector3 I_n = (s_b_hat * static_cast<Scalar>(-1.0)).cwiseMax(static_cast<Scalar>(0.0)) * I_max;

    PlantParam::Vector6 I_css;
    I_css.setSegment(0, I_p);
    I_css.setSegment(3, I_n);

    PlantParam::Vector6 y_css = I_css + beta_css + sigma_css.cwiseProduct(randn<6>());
    return y_css;
}

SensorsClass::Vector3 SensorsClass::magnetometer(const StateVector& states) {
    // model magnetometer 
    Quat q = states.segment<4>(6);
    Vector3 R = states.segment<3>(0);

    // get utc time 
    TimeReal jd = helpers.julianDate(current_time);

    // convert position to ecef 
    Vector3 R_ecef = helpers.eci2ecef(R, jd);

    // convert position to geodetic coordinates 
    HelperFunctions::Ecef2llaOutput lla = helpers.ecef2lla(R_ecef);

    // Compute decimal year from Julian Date using TimeReal precision
    TimeReal decYearTime = static_cast<TimeReal>(2000.0) + (jd - static_cast<TimeReal>(2451545.0)) / static_cast<TimeReal>(365.25);
    Scalar decYear = static_cast<Scalar>(decYearTime);

    // calculate magnetic field vector 
    Vector3 B_NED = helpers.wrldmagm(lla.lat, lla.lon, lla.alt, decYear);

    // Rotation matrix from NED to ecef 
    PlantParam::Matrix3 C_ned2ecef = helpers.dcmecef2ned(lla.lat, lla.lon).transpose();

    // Rotation matrix from ecef to eci 
    PlantParam::Matrix3 C_ecef2eci = helpers.dcmeci2ecef(jd).transpose();

    // Rotate magnetic field from NED to ECI frame
    Vector3 B_ECI = C_ecef2eci * (C_ned2ecef * B_NED);

    // passive rotation (i-->b) of magnetic vector 
    Quat q_star = helpers.quatconj(q);
    Vector3 B_b = helpers.quatRotate(q_star, B_ECI);

    // add uncertainty to body vector
    Vector3 y_B = B_b + beta_mag + sigma_mag.cwiseProduct(randn<3>());

    return y_B;
}

PlantParam::Vector6 SensorsClass::gps(const StateVector& states) {
    // model gps 
    Vector3 R = states.segment<3>(0);
    Vector3 V = states.segment<3>(3);

    // get current time and convert to juliandate
    TimeReal jd = helpers.julianDate(current_time);

    // Rotation matrix from eci to ecef 
    PlantParam::Matrix3 C_eci2ecef = helpers.dcmeci2ecef(jd);

    // position in ecef frame 
    Vector3 R_ecef = C_eci2ecef * R;

    // velocity in ecef frame 
    Vector3 V_ecef = C_eci2ecef * V - omega_earth.cross(R_ecef);

    // Vector 1st-order Gauss-Markov for position error (per axis)
    Scalar phi = std::exp(-K_gps * T_gps);
    nu = nu * phi + sigma_cep.cwiseProduct(randn<3>()) * std::sqrt(static_cast<Scalar>(1) - phi * phi);

    Vector3 y_R = R_ecef + nu;

    // Velocity noise (3x1) 
    Vector3 y_V = V_ecef + sigma_V.cwiseProduct(randn<3>());

    PlantParam::Vector6 y_gps;
    y_gps.setSegment(0, y_R);
    y_gps.setSegment(3, y_V);

    return y_gps;
}

SensorsClass::Vector4 SensorsClass::reaction_wheels(const StateVector& states) {
    // 1. Extract True Wheel Speeds (Indices 13-16)
    Vector4 omega_w = states.segment<4>(13);

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
