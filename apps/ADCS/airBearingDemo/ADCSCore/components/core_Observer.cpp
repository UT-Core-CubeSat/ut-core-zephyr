#include "core_Observer.hpp"

namespace {

ObserverClass::Vector3 quatRotateLocal(const ObserverClass::Quat& q,
                                       const ObserverClass::Vector3& v) {
    // Direct rotation formula: v' = v + 2*q_w*(q_v x v) + 2*(q_v x (q_v x v))
    ObserverClass::Scalar qw = q(0);
    ObserverClass::Vector3 qv = q.segment<3>(1);

    ObserverClass::Vector3 qv_cross_v = qv.cross(v);
    ObserverClass::Vector3 qv_cross_qv_cross_v = qv.cross(qv_cross_v);

    return v + static_cast<ObserverClass::Scalar>(2.0) * qw * qv_cross_v
             + static_cast<ObserverClass::Scalar>(2.0) * qv_cross_qv_cross_v;
}

} // namespace

ObserverClass::ObserverClass()
    : last_q_star(Param::Vector4::Zero()),
      q_hat(Param::Vector4::Zero()),
      beta_hat(Param::Observer::beta_gyro),
      tau_bias(Param::Observer::tau_bias),
      g_ref(Param::Observer::g_ref),
      B_ref(Param::Observer::B_ref),
      R_accel(Param::Observer::R_accel),
      R_mag(Param::Observer::R_mag),
      accel_min_norm(Param::Observer::accel_min_norm),
      mag_min_norm(Param::Observer::mag_min_norm),
      P(Param::Observer::P_0),
      G(Param::Observer::G),
      Q(Param::Observer::Q),
      sigma_v(Param::Observer::sigma_gyro),
      sigma_u(Param::Observer::sigma_bias_walk),
      epoch_time(static_cast<Param::TimeReal>(0.0)),
      current_time(static_cast<Param::TimeReal>(0.0))
{
    q_hat(0) = 1; q_hat(1) = 0; q_hat(2) = 0; q_hat(3) = 0;
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
    Scalar exp_term = static_cast<Scalar>(std::exp(-2.0f * dt / tau_bias));
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

    // Adaptive R: |accel| deviating from g means kinematic acceleration is
    // present (rotating / moving the board). Lean on gyro during motion by
    // inflating R_accel; keep it tight when truly at rest.
    constexpr Scalar kG = static_cast<Scalar>(9.80665);
    Scalar accel_err = std::abs(accel_norm - kG) / kG;       // fractional deviation
    constexpr Scalar kMotionInflate = static_cast<Scalar>(400.0); // R scales up to R*(1+400*err^2)
    Scalar R_scale = static_cast<Scalar>(1.0) + kMotionInflate * accel_err * accel_err;
    Param::Matrix3 R_accel_effective = R_accel * R_scale;
    
    // Predicted "up" direction in body frame from current estimate
    Quat q_conj;
    q_conj(0) = q_hat(0);
    q_conj.setSegment(1, -q_hat.segment<3>(1));
    Vector3 g_pred_body = quatRotateLocal(q_conj, g_ref);
    
    // Subtraction residual: z = g_meas - g_pred
    Vector3 z = g_meas_body - g_pred_body;

    // Measurement matrix H = -skew(g_pred)
    Param::Matrix36 H;
    H(0,0) = 0;                H(0,1) = -g_pred_body(2); H(0,2) = g_pred_body(1);
    H(1,0) = g_pred_body(2);   H(1,1) = 0;               H(1,2) = -g_pred_body(0);
    H(2,0) = -g_pred_body(1);  H(2,1) = g_pred_body(0);  H(2,2) = 0;
    H(0,3) = 0; H(0,4) = 0; H(0,5) = 0;
    H(1,3) = 0; H(1,4) = 0; H(1,5) = 0;
    H(2,3) = 0; H(2,4) = 0; H(2,5) = 0;
    
    // Kalman gain: K = P * H' * (H * P * H' + R)^-1
    Param::Matrix3 S_innov = H * P * H.transpose() + R_accel_effective;
    Param::Matrix3 S_inv = Math::inverse3x3(S_innov);

    // Chi-square outlier rejection (~3σ, 3 dof). Catches table bumps / vibration spikes.
    {
        Vector3 Sinv_z = S_inv * z;
        Scalar mahal2 = z.dot(Sinv_z);
        constexpr Scalar kChi2 = static_cast<Scalar>(14.16);
        if (mahal2 > kChi2) {
            return;  // diagnostic already stored above
        }
    }

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
    P = IKH * P * IKH.transpose() + K * R_accel_effective * K.transpose();
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
    Vector3 B_pred_body = quatRotateLocal(q_conj, B_ref);
    
    // Subtraction residual: z = B_meas - B_pred
    Vector3 z = B_meas_body - B_pred_body;

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

    // Chi-square outlier rejection. Catches mag spikes (phone walk-by, ferrous nearby).
    {
        Vector3 Sinv_z = S_inv * z;
        Scalar mahal2 = z.dot(Sinv_z);
        constexpr Scalar kChi2 = static_cast<Scalar>(14.16);
        if (mahal2 > kChi2) {
            return;
        }
    }

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
    Vector3 mag_meas = measurements.segment<3>(6);
    Param::Vector4 omega_w = measurements.segment<4>(9);

    // 0. Self-init from static samples (TRIAD).  While collecting, return identity
    //    attitude and zero rate so the controller's entry guard knows we are warming up.
    if (init_phase_ == InitPhase::Collecting) {
        accumulateInitSample(accel, omega_gyro, mag_meas);
        if (init_samples_collected_ >= kInitSampleTarget) {
            initializeFromStatic();
        }
        StateVector s;
        s.setSegment(0, q_hat);
        s.setSegment(4, Vector3::Zero());
        s.setSegment(7, omega_w);
        return s;
    }

    // 1. Propagate with gyro
    propagate(omega_gyro, dt);

    // 2. Correct with accelerometer (gravity vector)
    updateWithGravity(accel, dt);

    // 3. Correct with magnetometer (magnetic field vector).
    //    Disabled outside the Helmholtz cage: indoor field distortion drags
    //    yaw to different equilibria after any large motion.
    if (Param::Observer::use_magnetometer) {
        updateWithMagnetometer(mag_meas, dt);
    }

    // 4. Bias-corrected rate
    Param::Vector3 omega_b_hat = omega_gyro - beta_hat;

    StateVector states_hat;
    states_hat.setSegment(0, q_hat);
    states_hat.setSegment(4, omega_b_hat);
    states_hat.setSegment(7, omega_w);

    return states_hat;
}

void ObserverClass::accumulateInitSample(const Vector3& accel,
                                         const Vector3& gyro,
                                         const Vector3& mag) {
    // Reject samples with obvious motion or bad sensor data; restart on motion
    // so a bumped board cleanly re-bootstraps.
    if (gyro.norm() > kInitMotionGateRad) {
        init_samples_collected_ = 0;
        init_accel_sum_ = Vector3::Zero();
        init_gyro_sum_  = Vector3::Zero();
        init_mag_sum_   = Vector3::Zero();
        return;
    }
    if (accel.norm() < accel_min_norm) return;
    if (Param::Observer::use_magnetometer && mag.norm() < mag_min_norm) return;

    init_accel_sum_ = init_accel_sum_ + accel;
    init_gyro_sum_  = init_gyro_sum_  + gyro;
    init_mag_sum_   = init_mag_sum_   + mag;
    init_samples_collected_ += 1;
}

void ObserverClass::initializeFromStatic() {
    Scalar n = static_cast<Scalar>(init_samples_collected_);
    Vector3 a_body_avg = init_accel_sum_ / n;
    Vector3 g_body_avg = init_gyro_sum_  / n;
    Vector3 m_body_avg = init_mag_sum_   / n;

    // Static gyro mean → bias estimate (replaces hardcoded fallback).
    beta_hat = g_body_avg;

    Vector3 a_body_unit = a_body_avg.normalized();
    Vector3 m_body_unit = m_body_avg.normalized();

    if (Param::Observer::use_magnetometer) {
        // TRIAD: build q_hat consistent with measured gravity + measured heading.
        q_hat = triadAccelMag(a_body_unit, m_body_unit, g_ref);
        q_hat.normalize();
        // B_ref consistent with the just-computed attitude so the first mag
        // update has zero innovation. quatRotate(q_hat, body) → inertial.
        B_ref = quatRotateLocal(q_hat, m_body_unit);
    } else {
        // Mag fusion disabled (bench / no Helmholtz cage). Yaw is unobservable
        // from accel alone, so pin yaw=0 and use a pure tilt quaternion derived
        // from the measured gravity direction. This makes the boot pose the
        // yaw reference; yaw will simply hold (no drift, no snap-back error).
        q_hat = tiltQuatFromGravity(a_body_unit, g_ref);
        q_hat.normalize();
    }

    init_phase_ = InitPhase::Running;
}

ObserverClass::Quat ObserverClass::triadAccelMag(const Vector3& a_body_unit,
                                                  const Vector3& m_body_unit,
                                                  const Vector3& g_ref_inertial) {
    Quat q_id;
    q_id(0) = static_cast<Scalar>(1.0);
    q_id(1) = static_cast<Scalar>(0.0);
    q_id(2) = static_cast<Scalar>(0.0);
    q_id(3) = static_cast<Scalar>(0.0);

    // Body triad: b1 along measured gravity, b2 ⟂ to gravity in the (g, m) plane.
    Vector3 b1 = a_body_unit;
    Vector3 b2_raw = b1.cross(m_body_unit);
    if (b2_raw.norm() < static_cast<Scalar>(1e-6)) {
        return q_id;  // mag parallel to gravity — heading ambiguous
    }
    Vector3 b2 = b2_raw.normalized();
    Vector3 b3 = b1.cross(b2);

    // Inertial triad: i1 = g_ref; i2 chosen orthogonal to i1 (the resulting
    // heading axis is arbitrary, but B_ref will be redefined consistently
    // afterward so subsequent updates work correctly).
    Vector3 i1 = g_ref_inertial.normalized();
    Vector3 tmp = (std::abs(i1(0)) < static_cast<Scalar>(0.9))
                ? Vector3{static_cast<Scalar>(1.0), static_cast<Scalar>(0.0), static_cast<Scalar>(0.0)}
                : Vector3{static_cast<Scalar>(0.0), static_cast<Scalar>(1.0), static_cast<Scalar>(0.0)};
    Vector3 i2 = i1.cross(tmp).normalized();
    Vector3 i3 = i1.cross(i2);

    // R_bi rotates body→inertial: R_bi * b_k = i_k for k=1,2,3.
    // Equivalently R_bi(r,c) = Σ_k i_k(r) * b_k(c).
    Param::Matrix3 R;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            R(r, c) = i1(r) * b1(c) + i2(r) * b2(c) + i3(r) * b3(c);
        }
    }
    return dcmToQuat(R);
}

ObserverClass::Quat ObserverClass::tiltQuatFromGravity(const Vector3& a_body_unit,
                                                        const Vector3& g_ref_inertial) {
    // Shortest-arc quaternion q such that quatRotate(q_conj, g_ref) == a_body_unit
    // (matches observer convention: q is inertial-from-body).
    // Derivation: q rotates a_body into g_ref → axis = a × g, angle = acos(a·g).
    Quat q;
    Vector3 a = a_body_unit;
    Vector3 g = g_ref_inertial.normalized();
    Scalar d = a.dot(g);

    if (d > static_cast<Scalar>(0.999999)) {
        // Already aligned → identity
        q(0) = static_cast<Scalar>(1.0);
        q(1) = static_cast<Scalar>(0.0);
        q(2) = static_cast<Scalar>(0.0);
        q(3) = static_cast<Scalar>(0.0);
        return q;
    }
    if (d < static_cast<Scalar>(-0.999999)) {
        // Opposite → 180° about any axis orthogonal to g
        Vector3 tmp = (std::abs(g(0)) < static_cast<Scalar>(0.9))
                    ? Vector3{static_cast<Scalar>(1.0), static_cast<Scalar>(0.0), static_cast<Scalar>(0.0)}
                    : Vector3{static_cast<Scalar>(0.0), static_cast<Scalar>(1.0), static_cast<Scalar>(0.0)};
        Vector3 axis = g.cross(tmp).normalized();
        q(0) = static_cast<Scalar>(0.0);
        q(1) = axis(0); q(2) = axis(1); q(3) = axis(2);
        return q;
    }

    Vector3 axis = a.cross(g);            // body a rotated to inertial g
    Scalar w = static_cast<Scalar>(1.0) + d;
    q(0) = w;
    q(1) = axis(0); q(2) = axis(1); q(3) = axis(2);
    q.normalize();
    return q;
}

ObserverClass::Quat ObserverClass::dcmToQuat(const Param::Matrix3& R) {
    // Shepperd's method: pick the largest of {1+tr, 1+R00-R11-R22, ...} for numerical stability.
    Quat q;
    Scalar tr = R(0,0) + R(1,1) + R(2,2);
    if (tr > static_cast<Scalar>(0.0)) {
        Scalar s = std::sqrt(tr + static_cast<Scalar>(1.0)) * static_cast<Scalar>(2.0);
        q(0) = static_cast<Scalar>(0.25) * s;
        q(1) = (R(2,1) - R(1,2)) / s;
        q(2) = (R(0,2) - R(2,0)) / s;
        q(3) = (R(1,0) - R(0,1)) / s;
    } else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) {
        Scalar s = std::sqrt(static_cast<Scalar>(1.0) + R(0,0) - R(1,1) - R(2,2)) * static_cast<Scalar>(2.0);
        q(0) = (R(2,1) - R(1,2)) / s;
        q(1) = static_cast<Scalar>(0.25) * s;
        q(2) = (R(0,1) + R(1,0)) / s;
        q(3) = (R(0,2) + R(2,0)) / s;
    } else if (R(1,1) > R(2,2)) {
        Scalar s = std::sqrt(static_cast<Scalar>(1.0) + R(1,1) - R(0,0) - R(2,2)) * static_cast<Scalar>(2.0);
        q(0) = (R(0,2) - R(2,0)) / s;
        q(1) = (R(0,1) + R(1,0)) / s;
        q(2) = static_cast<Scalar>(0.25) * s;
        q(3) = (R(1,2) + R(2,1)) / s;
    } else {
        Scalar s = std::sqrt(static_cast<Scalar>(1.0) + R(2,2) - R(0,0) - R(1,1)) * static_cast<Scalar>(2.0);
        q(0) = (R(1,0) - R(0,1)) / s;
        q(1) = (R(0,2) + R(2,0)) / s;
        q(2) = (R(1,2) + R(2,1)) / s;
        q(3) = static_cast<Scalar>(0.25) * s;
    }
    return q;
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