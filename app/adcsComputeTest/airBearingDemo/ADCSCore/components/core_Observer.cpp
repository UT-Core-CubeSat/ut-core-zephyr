#include "core_Observer.hpp"
#include <iostream>  // DEBUG

ObserverClass::ObserverClass()
    : q_hat(Param::Vector4::Zero()),
      beta_hat(Param::Observer::beta_gyro),
      tau_bias(Param::Observer::tau_bias),
      P(Param::Observer::P_0),
      G(Param::Observer::G),
      Q(Param::Observer::Q),
      sigma_v(Param::Observer::sigma_gyro),
      sigma_u(Param::Observer::sigma_bias_walk),
      epoch_time(static_cast<Param::TimeReal>(0.0)), 
      current_time(static_cast<Param::TimeReal>(0.0)),
      last_q_star(Param::Vector4::Zero()),
    g_ref(Param::Observer::g_ref),
    B_ref(Param::Observer::B_ref),
    R_accel(Param::Observer::R_accel),
    R_mag(Param::Observer::R_mag),
    accel_min_norm(Param::Observer::accel_min_norm),
    mag_min_norm(Param::Observer::mag_min_norm)
{
    q_hat(0) = 1; q_hat(1) = 0; q_hat(2) = 0; q_hat(3) = 0;
    last_accel_innovation = Vector3::Zero();
    last_accel_innovation_norm = static_cast<Scalar>(0.0);
    last_mag_innovation = Vector3::Zero();
    last_mag_innovation_norm = static_cast<Scalar>(0.0);
}

void ObserverClass::propagate(const Vector3& omega_meas, Scalar dt) {
    Vector3 omega_hat = omega_meas - beta_hat;

    Vector3 q_vec = q_hat.segment<3>(1);
    Scalar q0 = q_hat(0);

    Vector3 q_dot_vec = static_cast<Scalar>(0.5)*(q0*omega_hat + q_vec.cross(omega_hat));
    Scalar q_dot_0 = static_cast<Scalar>(-0.5) * q_vec.dot(omega_hat);

    Quat q_dot;
    q_dot(0) = q_dot_0;
    q_dot.setSegment(1, q_dot_vec);

    q_hat += q_dot * dt;
    q_hat.normalize();

    // Covariance propagation (discrete-time formulation)
    // State transition matrix F (continuous)
    Param::Matrix6 F;
    F(0,0) = 0;              F(0,1) = omega_hat(2);  F(0,2) = -omega_hat(1); F(0,3) = -1; F(0,4) = 0;  F(0,5) = 0;
    F(1,0) = -omega_hat(2);  F(1,1) = 0;             F(1,2) = omega_hat(0);  F(1,3) = 0;  F(1,4) = -1; F(1,5) = 0;
    F(2,0) = omega_hat(1);   F(2,1) = -omega_hat(0); F(2,2) = 0;             F(2,3) = 0;  F(2,4) = 0;  F(2,5) = -1;
    F(3,0) = 0; F(3,1) = 0; F(3,2) = 0; F(3,3) = -1/tau_bias; F(3,4) = 0; F(3,5) = 0;
    F(4,0) = 0; F(4,1) = 0; F(4,2) = 0; F(4,3) = 0; F(4,4) = -1/tau_bias; F(4,5) = 0;
    F(5,0) = 0; F(5,1) = 0; F(5,2) = 0; F(5,3) = 0; F(5,4) = 0; F(5,5) = -1/tau_bias;

    // Discrete state transition: Phi ≈ I + F*dt
    Param::Matrix6 I6 = Param::Matrix6::Identity();
    Param::Matrix6 Phi = I6 + F * dt;
    
    // Discrete process noise: Qd
    Param::Matrix6 Qd = Param::Matrix6::Zero();
    
    // Attitude part (top-left 3x3): Simplified from G.block * Q.block * G.block.transpose() * dt
    // Since G.block is -I and Q.block is sigma^2 I, this reduces to sigma^2 I * dt
    Qd(0,0) = sigma_v(0) * sigma_v(0) * dt;
    Qd(1,1) = sigma_v(1) * sigma_v(1) * dt;
    Qd(2,2) = sigma_v(2) * sigma_v(2) * dt;
    
    // Bias part (bottom-right 3x3): Gauss-Markov as before
    Scalar exp_term = static_cast<Scalar>(std::exp(-2.0 * dt / tau_bias));
    Scalar bias_qd = (sigma_u(0) * sigma_u(0)) * (static_cast<Scalar>(1.0) - exp_term);
    Qd(3,3) = bias_qd;
    Qd(4,4) = bias_qd;
    Qd(5,5) = bias_qd;
    
    // Discrete covariance propagation: P = Phi * P * Phi' + Qd
    P = Phi * P * Phi.transpose() + Qd;
    P = static_cast<Scalar>(0.5) * (P + P.transpose());
}

void ObserverClass::updateWithGravity(const Vector3& accel_meas, Scalar dt) {
    // =====================================================================
    // MEKF correction using gravity vector (standard formulation)
    // =====================================================================
    
    // Normalize measured acceleration (gravity direction in body)
    Scalar accel_norm = accel_meas.norm();
    if (accel_norm < accel_min_norm) {
        return;  // Skip update if acceleration too small
    }
    
    // Measured "up" direction in body frame (normalized)
    Vector3 g_meas_body = accel_meas / accel_norm;
    
    // Predicted "up" direction in body frame from current estimate
    Quat q_conj;
    q_conj(0) = q_hat(0);
    q_conj.setSegment(1, -q_hat.segment<3>(1));
    Vector3 g_pred_body = helpers.quatRotate(q_conj, g_ref);
    
    // Subtraction residual: z = g_meas - g_pred
    Vector3 z = g_meas_body - g_pred_body;
    
    // Store innovation for diagnostics
    last_accel_innovation = z;
    last_accel_innovation_norm = z.norm();
    
    // Measurement matrix H = -skew(g_pred)
    Param::Matrix36 H;
    H(0,0) = 0;                H(0,1) = -g_pred_body(2); H(0,2) = g_pred_body(1);
    H(1,0) = g_pred_body(2);   H(1,1) = 0;               H(1,2) = -g_pred_body(0);
    H(2,0) = -g_pred_body(1);  H(2,1) = g_pred_body(0);  H(2,2) = 0;
    H(0,3) = 0; H(0,4) = 0; H(0,5) = 0;
    H(1,3) = 0; H(1,4) = 0; H(1,5) = 0;
    H(2,3) = 0; H(2,4) = 0; H(2,5) = 0;
    
    // Kalman gain: K = P * H' * (H * P * H' + R)^-1
    Param::Matrix3 S_innov = H * P * H.transpose() + R_accel;
    Param::Matrix3 S_inv = Math::inverse3x3(S_innov);
    Param::Matrix63 K = P * H.transpose() * S_inv;
    
    // State correction
    Param::Vector6 dx = K * z;
    Vector3 dtheta = dx.segment<3>(0);
    Vector3 dbeta = dx.segment<3>(3);
    
    // Apply quaternion correction (small angle)
    Quat dq;
    dq(0) = static_cast<Scalar>(1.0);
    dq.setSegment(1, static_cast<Scalar>(0.5) * dtheta);
    dq.normalize();
    
    q_hat = quatMultiply(dq, q_hat);
    q_hat.normalize();
    
    // Update bias estimate
    beta_hat += dbeta;
    
    // Update covariance: Joseph form P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
    Param::Matrix6 I6 = Param::Matrix6::Identity();
    Param::Matrix6 IKH = I6 - K * H;
    P = IKH * P * IKH.transpose() + K * R_accel * K.transpose();
    P = static_cast<Scalar>(0.5) * (P + P.transpose());
}

void ObserverClass::updateWithMagnetometer(const Vector3& mag_meas, Scalar dt) {
    // Normalize measured magnetic field
    Scalar mag_norm = mag_meas.norm();
    if (mag_norm < mag_min_norm) {
        return;  // Skip if no valid measurement
    }
    Vector3 B_meas_body = mag_meas / mag_norm;
    
    // Predicted B direction in body frame from current estimate
    Quat q_conj;
    q_conj(0) = q_hat(0);
    q_conj.setSegment(1, -q_hat.segment<3>(1));
    Vector3 B_pred_body = helpers.quatRotate(q_conj, B_ref);
    
    // Subtraction residual: z = B_meas - B_pred
    Vector3 z = B_meas_body - B_pred_body;
    
    // Store innovation for diagnostics
    last_mag_innovation = z;
    last_mag_innovation_norm = z.norm();
    
    // Measurement matrix H = -skew(B_pred)
    Param::Matrix36 H;
    H(0,0) = 0;                H(0,1) = -B_pred_body(2); H(0,2) = B_pred_body(1);
    H(1,0) = B_pred_body(2);   H(1,1) = 0;               H(1,2) = -B_pred_body(0);
    H(2,0) = -B_pred_body(1);  H(2,1) = B_pred_body(0);  H(2,2) = 0;
    H(0,3) = 0; H(0,4) = 0; H(0,5) = 0;
    H(1,3) = 0; H(1,4) = 0; H(1,5) = 0;
    H(2,3) = 0; H(2,4) = 0; H(2,5) = 0;
    
    // Kalman gain
    Param::Matrix3 S_innov = H * P * H.transpose() + R_mag;
    Param::Matrix3 S_inv = Math::inverse3x3(S_innov);
    Param::Matrix63 K = P * H.transpose() * S_inv;
    
    // State correction
    Param::Vector6 dx = K * z;
    Vector3 dtheta = dx.segment<3>(0);
    Vector3 dbeta = dx.segment<3>(3);
    
    // Apply quaternion correction
    Quat dq;
    dq(0) = static_cast<Scalar>(1.0);
    dq.setSegment(1, static_cast<Scalar>(0.5) * dtheta);
    dq.normalize();
    
    q_hat = quatMultiply(dq, q_hat);
    q_hat.normalize();
    
    beta_hat += dbeta;
    
    // Update covariance: Joseph form P = (I - K*H)*P*(I - K*H)^T + K*R*K^T
    Param::Matrix6 I6 = Param::Matrix6::Identity();
    Param::Matrix6 IKH = I6 - K * H;
    P = IKH * P * IKH.transpose() + K * R_mag * K.transpose();
    P = static_cast<Scalar>(0.5) * (P + P.transpose());
}

ObserverClass::StateVector ObserverClass::update(const Param::Vector13& measurements, 
                                                  const TimeReal& t, 
                                                  const Scalar& dt) {
    current_time = t;

    Param::Vector3 accel = measurements.segment<3>(0);
    Param::Vector3 omega_gyro = measurements.segment<3>(3);
    Param::Vector4 omega_w = measurements.segment<4>(9);

    // 1. Propagate with gyro
    propagate(omega_gyro, dt);
    
    // 2. Correct with accelerometer (gravity vector)
    updateWithGravity(accel, dt);
    
    // 3. Correct with magnetometer (magnetic field vector)
    Vector3 mag_meas = measurements.segment<3>(6);
    updateWithMagnetometer(mag_meas, dt);

    // 3. Bias-corrected rate
    Param::Vector3 omega_b_hat = omega_gyro - beta_hat;
    
    // 3. Correct with magnetometer (magnetic field vector)
    StateVector states_hat;
    states_hat.setSegment(0, q_hat);
    states_hat.setSegment(4, omega_b_hat);
    states_hat.setSegment(7, omega_w);

    return states_hat;
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