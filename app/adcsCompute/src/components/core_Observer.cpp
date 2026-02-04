#include "core_Observer.hpp"


// Constructor 
ObserverClass::ObserverClass()
    : // Initialize Members
    q_hat(Param::Vector4::Zero()),
    beta_hat(Param::Observer::beta_gyro),
    P(Param::Observer::P_0),
    G(Param::Observer::G),
    Q(Param::Observer::Q),
    sigma_v(Param::Observer::sigma_gyro),
    sigma_u(Param::Observer::sigma_bias_walk),
    R_star(Param::Observer::R_star),
    R_quest(Param::Observer::R_quest),
    Ts(Param::SimTime::Ts),
    omega_earth(Param::Observer::omega_earth),
    epoch_time(0.0), 
    current_time(0.0),
    use_star_tracker(true),
    use_magnetometer(false),
    use_sun_sensor(false),
    last_star_update_time(0.0),
    last_quest_update_time(0.0),
    T_star(Param::Observer::T_star),
    T_quest(Param::Observer::T_quest),  // Use same update rate as star tracker
    I_max(Param::Observer::I_max),
    last_q_star(Param::Vector4::Zero())  // Initialize to initial quaternion
{
    q_hat(0) = 1; q_hat(1) = 0; q_hat(2) = 0; q_hat(3) = 0; // Identity quaternion
}

void ObserverClass::propagate(const Vector3& omega_meas) {
    // Propagation Step 
    Vector3 omega_hat = omega_meas - beta_hat;

    // Quaternion kinematics 
    Vector3 q_vec = q_hat.segment<3>(1);
    Scalar q0 = q_hat(0);

    Vector3 q_dot_vec = 0.5*(q0*omega_hat + q_vec.cross(omega_hat));
    Scalar q_dot_0 = -0.5 * q_vec.dot(omega_hat);

    Quat q_dot;
    q_dot(0) = q_dot_0;
    q_dot.setSegment(1, q_dot_vec);

    // Integrate 
    q_hat += q_dot * Ts;
    q_hat.normalize();

    // Covariance propagation 
    Param::Matrix6 F;
    //F << -skew(omega_hat), -Param::Matrix3::Identity(),
    //    Param::Matrix3::Zero(), Param::Matrix3::Zero();
    F(0,0) = 0.0;          F(0,1) = omega_hat(2); F(0,2) = -omega_hat(1); F(0,3) = -1.0;    F(0,4) = 0.0;      F(0,5) = 0.0;
    F(1,0) = -omega_hat(2); F(1,1) = 0.0;        F(1,2) = omega_hat(0);  F(1,3) = 0.0;     F(1,4) = -1.0;     F(1,5) = 0.0;
    F(2,0) = omega_hat(1);  F(2,1) = -omega_hat(0); F(2,2) = 0.0;         F(2,3) = 0.0;     F(2,4) = 0.0;      F(2,5) = -1.0;
    F(3,0) = 0.0;          F(3,1) = 0.0;        F(3,2) = 0.0;         F(3,3) = 0.0;     F(3,4) = 0.0;      F(3,5) = 0.0;
    F(4,0) = 0.0;          F(4,1) = 0.0;        F(4,2) = 0.0;         F(4,3) = 0.0;     F(4,4) = 0.0;      F(4,5) = 0.0;
    F(5,0) = 0.0;          F(5,1) = 0.0;        F(5,2) = 0.0;         F(5,3) = 0.0;     F(5,4) = 0.0;      F(5,5) = 0.0;
    P = P + Ts * (F * P + P * F.transpose() + G * Q * G.transpose());
    P = 0.5 * (P + P.transpose()); // Ensure symmetry
}

void ObserverClass::star_tracker_update(const Quat& q_meas) {
    // Star Tracker Update 
    Quat q_hat_inv;
    q_hat_inv(0) = q_hat(0);
    q_hat_inv(1) = -q_hat(1);
    q_hat_inv(2) = -q_hat(2);
    q_hat_inv(3) = -q_hat(3);
    Quat delta_q = quatMultiply(q_hat_inv, q_meas);

    // Positive scalar part 
    if (delta_q(0) < 0) {
        delta_q = -delta_q;
    }

    // Measurement residual 
    Vector3 y_tilde = 2*delta_q.segment<3>(1);

    // Linearized model H 
    Param::Matrix36 H;
    //H << Param::Matrix3::Identity(), Param::Matrix3::Zero();
    H(0,0) = 1.0; H(0,1) = 0.0; H(0,2) = 0.0; H(0,3) = 0.0; H(0,4) = 0.0; H(0,5) = 0.0;
    H(1,0) = 0.0; H(1,1) = 1.0; H(1,2) = 0.0; H(1,3) = 0.0; H(1,4) = 0.0; H(1,5) = 0.0;
    H(2,0) = 0.0; H(2,1) = 0.0; H(2,2) = 1.0; H(2,3) = 0.0; H(2,4) = 0.0; H(2,5) = 0.0;
    Param::Matrix3 S = H*P*H.transpose() + R_star;
    Param::Matrix63 K = P*H.transpose()*Math::inverse3x3(S);

    // Kalman Correction
    Param::Vector6 delta_x = K * y_tilde;
    Vector3 delta_alpha = delta_x.segment<3>(0);
    Vector3 delta_beta = delta_x.segment<3>(3);

    // Joseph covariance update 
    Param::Matrix6 I_KH = Param::Matrix6::Identity() - K * H;
    P = I_KH * P * I_KH.transpose() + K * R_star * K.transpose();
    P = 0.5 * (P + P.transpose()); // Ensure symmetry

    // Quaternion correction 
    Quat dq_plus; 
    //dq_plus << 1.0, 0.5*delta_alpha;
    dq_plus(0) = 1.0;
    dq_plus(1) = 0.5 * delta_alpha(0);
    dq_plus(2) = 0.5 * delta_alpha(1);
    dq_plus(3) = 0.5 * delta_alpha(2);
    dq_plus.normalize();
    q_hat = quatMultiply(q_hat, dq_plus);
    q_hat.normalize();

    // Bias update 
    beta_hat += delta_beta;
}

void ObserverClass::quest_update(const Quat& q_meas) {
    // QUEST Update 
    Quat q_hat_inv;
    q_hat_inv(0) = q_hat(0);
    q_hat_inv(1) = -q_hat(1);
    q_hat_inv(2) = -q_hat(2);
    q_hat_inv(3) = -q_hat(3);
    Quat delta_q = quatMultiply(q_hat_inv, q_meas);

    // Positive scalar part 
    if (delta_q(0) < 0) {
        delta_q = -delta_q;
    }

    // Measurement residual 
    Vector3 y_tilde = 2*delta_q.segment<3>(1);

    // Linearized model H 
    Param::Matrix36 H;
    //H << Param::Matrix3::Identity(), Param::Matrix3::Zero();
    H(0,0) = 1.0; H(0,1) = 0.0; H(0,2) = 0.0; H(0,3) = 0.0; H(0,4) = 0.0; H(0,5) = 0.0;
    H(1,0) = 0.0; H(1,1) = 1.0; H(1,2) = 0.0; H(1,3) = 0.0; H(1,4) = 0.0; H(1,5) = 0.0;
    H(2,0) = 0.0; H(2,1) = 0.0; H(2,2) = 1.0; H(2,3) = 0.0; H(2,4) = 0.0; H(2,5) = 0.0;
    Param::Matrix3 S = H*P*H.transpose() + R_quest;
    Param::Matrix63 K = P*H.transpose()*Math::inverse3x3(S);

    // Kalman Correction
    Param::Vector6 delta_x = K * y_tilde;
    Vector3 delta_alpha = delta_x.segment<3>(0);
    Vector3 delta_beta = delta_x.segment<3>(3);

    // Joseph covariance update 
    Param::Matrix6 I_KH = Param::Matrix6::Identity() - K * H;
    P = I_KH * P * I_KH.transpose() + K * R_quest * K.transpose();
    P = 0.5 * (P + P.transpose()); // Ensure symmetry

    // Quaternion correction 
    Quat dq_plus; 
    //dq_plus << 1.0, 0.5*delta_alpha;
    dq_plus(0) = 1.0;
    dq_plus(1) = 0.5 * delta_alpha(0);
    dq_plus(2) = 0.5 * delta_alpha(1);
    dq_plus(3) = 0.5 * delta_alpha(2);
    dq_plus.normalize();
    q_hat = quatMultiply(q_hat, dq_plus);
    q_hat.normalize();

    // Bias update 
    beta_hat += delta_beta;
}

ObserverClass::Quat ObserverClass::quest_algorithm(const Scalar& JD, 
                                     const Vector3& r_eci, 
                                     const Vector3& b_sun, 
                                     const Vector3& b_mag) {
    // Inertial ref vectors 
    Vector3 r_sun = helpers.earth2sun(JD);
    Vector3 r_mag = compute_r_mag(JD, r_eci);

    // Normalize body-frame sensor vectors 
    Vector3 b_sun_norm = b_sun.normalized();
    Vector3 b_mag_norm = b_mag.normalized();

    // Deifne weights 
    Scalar a_sun = 0.6;
    Scalar a_mag = 0.4;

    // Construct B Matrix 
    Param::Matrix3 B = a_sun * (Math::outerProduct(b_sun_norm,r_sun)) + a_mag * (Math::outerProduct(b_mag_norm,r_mag));

    // Compute K Matrix 
    Param::Matrix3 S = B + B.transpose();
    Scalar sigma = B.trace();
    Param::Vector3 Z;
    //Z << B(1,2) - B(2,1),
    //     B(2,0) - B(0,2),
    //     B(0,1) - B(1,0);
    Z(0) = B(1,2) - B(2,1);
    Z(1) = B(2,0) - B(0,2);
    Z(2) = B(0,1) - B(1,0);

    Param::Matrix4 K; 
    //K << sigma,        Z.transpose(),
    //     Z,      S - sigma * Param::Matrix3::Identity();
    K(0,0) = sigma;    K(0,1) = Z(0);    K(0,2) = Z(1);    K(0,3) = Z(2);
    K(1,0) = Z(0);     K(1,1) = S(0,0) - sigma; K(1,2) = S(0,1);    K(1,3) = S(0,2);
    K(2,0) = Z(1);     K(2,1) = S(1,0);    K(2,2) = S(1,1) - sigma; K(2,3) = S(1,2);
    K(3,0) = Z(2);     K(3,1) = S(2,0);    K(3,2) = S(2,1);    K(3,3) = S(2,2) - sigma;

    // Compute eigenvector of K with max eigenvalue 
    Param::Vector4 q_meas = Math::maxEigenvector4x4Symmetric(K);

    // Ensure positive scalar part 
    if (q_meas(0) < 0) {
        q_meas = -q_meas;
    }

    q_meas = q_meas / q_meas.norm();
    Quat q_quest = q_meas;
    return q_quest;
}



ObserverClass::StateVector ObserverClass::update(const Param::Vector29& measurements, const Scalar& t) {
    // Compute current Unix timestamp from epoch + simulation time
    current_time = t;

    // 1. Unpack Measurements (0-based indexing!)
    // MATLAB 4:6   -> C++ segment<3>(3)
    // MATLAB 7:10  -> C++ segment<4>(6)
    // MATLAB 11:16 -> C++ segment<6>(10)
    // MATLAB 20:25 -> C++ segment<6>(19)
    // MATLAB 26:29 -> C++ segment<4>(25)
    
    Param::Vector3 omega_gyro = measurements.segment<3>(3);
    Param::Vector4 q_star     = measurements.segment<4>(6);
    Param::Vector6 y_css      = measurements.segment<6>(10);
    Param::Vector6 gps_ecef   = measurements.segment<6>(19);
    Param::Vector4 omega_w    = measurements.segment<4>(25);

    // ================================================================
    // 1. Propagate attitude and covariance
    // ================================================================
    propagate(omega_gyro);

    // ================================================================
    // 2. Determine which attitude source to use
    // ================================================================
    
    // Initialize last_q_star if it's all zeros (first run check)
    if (last_q_star.isZero()) {
        last_q_star = q_star;
    }

    Param::Real q_diff = (q_star - last_q_star).norm();
    
    // Check for NaNs and change in value
    bool valid_star_measurement = !q_star.hasNaN(); 
    bool new_star_update = (q_diff > 1e-6) && valid_star_measurement;

    // --- Sun sensor processing ---
    // (Assuming synthesize_sun_vector is a private helper you implemented)
    Param::Vector3 b_sun = synthesize_sun_vector(y_css, I_max);
    b_sun.normalize();
    
    // Check Eclipse (Sum of all CSS currents)
    Param::Real sun_current_sum = y_css.sum();
    Param::Real eclipse_threshold = 0.05 * I_max * 6.0;
    bool in_eclipse = (sun_current_sum < eclipse_threshold);

    // --- Attitude update logic ---
    if (use_star_tracker && new_star_update) {
        star_tracker_update(q_star);
        last_q_star = q_star;
        last_star_update_time = current_time;
    } else if (!in_eclipse && (current_time - last_quest_update_time >= T_quest)) {
        Param::Real jd = helpers.julianDate(current_time);
        
        Param::Vector3 b_mag = measurements.segment<3>(16);
        
        // Check that sun and mag vectors have valid norms before QUEST
        Param::Real b_sun_norm = b_sun.norm();
        Param::Real b_mag_norm = b_mag.norm();
        
        if (b_sun_norm > 1e-6 && b_mag_norm > 1e-6) {
            b_sun = b_sun / b_sun_norm;  // Safe normalization
            b_mag = b_mag / b_mag_norm;  // Safe normalization
            
            Param::Matrix3 C_eci2ecef = helpers.dcmeci2ecef(jd);
            Param::Vector3 r_ecef = gps_ecef.head<3>();
            Param::Vector3 r_eci = C_eci2ecef.transpose() * r_ecef;
        
            Param::Vector4 q_quest = quest_algorithm(jd, r_eci, b_sun, b_mag);
            quest_update(q_quest);
            last_quest_update_time = current_time;
        }
        // else: Skip QUEST this cycle if sensor data is invalid
    }
    // else: in eclipse with no star tracker — propagation only

    // ================================================================
    // 3. Transform position/velocity to ECI for output
    // ================================================================
    // else: in eclipse with no star tracker — propagation only

    // ================================================================
    // 3. Transform position/velocity to ECI for output
    // ================================================================
    // Re-calculate JD only if necessary (optimization: reuse 'jd' from above if scope allows)
    Param::Real jd_out = helpers.julianDate(current_time);
    Param::Matrix3 C_eci2ecef_out = helpers.dcmeci2ecef(jd_out);
    
    Param::Vector3 r_ecef_final = gps_ecef.head<3>();
    Param::Vector3 v_ecef_final = gps_ecef.tail<3>();

    // Position: Simple Rotation
    Param::Vector3 R_hat = C_eci2ecef_out.transpose() * r_ecef_final;

    // Velocity: Rotation + Coriolis Effect (Transport Theorem)
    // V_i = C^T * V_b + omega x R_i
    Param::Vector3 V_hat = C_eci2ecef_out.transpose() * v_ecef_final + 
                           Param::Observer::omega_earth.cross(R_hat);

    // Calculate Bias-Corrected Rate
    Param::Vector3 omega_b_hat = omega_gyro - beta_hat;
    
    // Normalize Quaternion Estimate
    q_hat.normalize();

    // Pack Output State Vector (17x1)
    StateVector states_hat;
    //states_hat << R_hat,       // 0-2
    //              V_hat,       // 3-5
    //              q_hat,       // 6-9
    //              omega_b_hat, // 10-12
    //              omega_w;     // 13-16
    states_hat.setSegment(0, R_hat);          // 0-2
    states_hat.setSegment(3, V_hat);          // 3-5
    states_hat.setSegment(6, q_hat);          // 6-9
    states_hat.setSegment(10, omega_b_hat);   // 10-12
    states_hat.setSegment(13, omega_w);       // 13-16

    return states_hat;
}

ObserverClass::Vector3 ObserverClass::compute_r_mag(const Scalar& JD, 
                                                 const Vector3& r_eci) {
    // Compute position in ECEF frame
    Vector3 r_ecef = helpers.eci2ecef(r_eci, JD);
    
    // Convert ECEF to Geodetic Coordinates (lat, lon, alt)
    HelperFunctions::Ecef2llaOutput lla = helpers.ecef2lla(r_ecef);
    Scalar lat = lla.lat;
    Scalar lon = lla.lon;
    Scalar alt = lla.alt;

    // Compute decimal year from Julian Date
    Scalar decYear = 2000.0 + (JD - 2451545.0) / 365.25;

    // Compute magnetic field in NED frame using WMM
    Vector3 B_ned = helpers.wrldmagm(lat, lon, alt, decYear);

    // Convert NED to ECEF
    Param::Matrix3 dcm_ecef2ned = helpers.dcmecef2ned(lat, lon);
    Param::Matrix3 dcm_ned2ecef = dcm_ecef2ned.transpose();
    Param::Vector3 B_ecef = dcm_ned2ecef * B_ned;

    // Convert ECEF to ECI
    Param::Matrix3 dcm_eci2ecef = helpers.dcmeci2ecef(JD);
    Param::Matrix3 dcm_ecef2eci = dcm_eci2ecef.transpose();
    Param::Vector3 B_eci = dcm_ecef2eci * B_ecef;

    return B_eci;
}

Param::Matrix3 ObserverClass::skew(const Vector3& v) {
    Param::Matrix3 S;
    S(0,0) = 0.0;    S(0,1) = -v(2); S(0,2) = v(1);
    S(1,0) = v(2);   S(1,1) = 0.0;    S(1,2) = -v(0);
    S(2,0) = -v(1);  S(2,1) = v(0);   S(2,2) = 0.0;
    return S;
}

ObserverClass::Quat ObserverClass::quatMultiply(const Quat& q1, const Quat& q2) {
    Quat q_result;
    q_result(0) = q1(0)*q2(0) - q1.segment<3>(1).dot(q2.segment<3>(1));
    Vector3 vec_part = q1(0)*q2.segment<3>(1) + q2(0)*q1.segment<3>(1) + q1.segment<3>(1).cross(q2.segment<3>(1));
    q_result.setSegment(1, vec_part);
    return q_result;
}

ObserverClass::Vector3 ObserverClass::synthesize_sun_vector(const Param::Vector6& y_css,
                                                             const Scalar& I_max) {
    // Face ordering assumed: +X,+Y,+Z,-X,-Y,-Z
    Vector3 I_diff;
    I_diff(0) = y_css(0) - y_css(3); // +X minus -X
    I_diff(1) = y_css(1) - y_css(4); // +Y minus -Y
    I_diff(2) = y_css(2) - y_css(5); // +Z minus -Z
    Vector3 s_est = I_diff / I_max; // Eq. (4.21) linear relation

    // Normalize if we have a meaningful signal; otherwise mark invalid
    if (s_est.norm() > 1e-8) {
        s_est.normalize();         // Eq. (4.22)
    } else {
        s_est = Vector3::Constant(std::numeric_limits<Scalar>::quiet_NaN()); // Eclipse / no sun
    }

    return s_est;
}