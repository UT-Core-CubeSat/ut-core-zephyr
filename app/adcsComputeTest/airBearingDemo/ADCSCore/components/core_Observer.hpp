#ifndef CORE_OBSERVER_HPP
#define CORE_OBSERVER_HPP

#include "core_Parameters.hpp"

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

    // Warmup diagnostics (TRIAD self-init)
    bool isRunning() const { return init_phase_ == InitPhase::Running; }
    int  getInitSamplesCollected() const { return init_samples_collected_; }
    static constexpr int getInitSampleTarget() { return kInitSampleTarget; }
    
private:
    void propagate(const Vector3& omega_meas, Scalar dt);
    void updateWithGravity(const Vector3& accel_meas, Scalar dt);  // NEW
    void updateWithMagnetometer(const Vector3& mag_meas, Scalar dt);  // NEW
    Param::Matrix3 skew(const Vector3& v);
    Quat quatMultiply(const Quat& q1, const Quat& q2);

    // --- Self-initialization (TRIAD-based) ---
    enum class InitPhase { Collecting, Running };
    InitPhase init_phase_ = InitPhase::Collecting;
    static constexpr int kInitSampleTarget = 120;        // ~2 s @ 60 Hz
    static constexpr Scalar kInitMotionGateRad =
            static_cast<Scalar>(0.08);                   // ~4.6 deg/s — tolerates I3G4250D single-sample noise
    int init_samples_collected_ = 0;
    Vector3 init_accel_sum_ = Vector3::Zero();
    Vector3 init_gyro_sum_  = Vector3::Zero();
    Vector3 init_mag_sum_   = Vector3::Zero();

    void accumulateInitSample(const Vector3& accel,
                              const Vector3& gyro,
                              const Vector3& mag);
    void initializeFromStatic();
    static Quat triadAccelMag(const Vector3& a_body_unit,
                              const Vector3& m_body_unit,
                              const Vector3& g_ref_inertial);
    static Quat tiltQuatFromGravity(const Vector3& a_body_unit,
                                    const Vector3& g_ref_inertial);
    static Quat dcmToQuat(const Param::Matrix3& R);

    Quat last_q_star;
    Quat q_hat;
    Vector3 beta_hat;
    Scalar tau_bias;
    Vector3 g_ref;
    Vector3 B_ref;
    Param::Matrix3 R_accel;
    Param::Matrix3 R_mag;
    Scalar accel_min_norm;
    Scalar mag_min_norm;
    Param::Matrix6 P;
    Param::Matrix6 G;
    Param::Matrix6 Q;
    Vector3 sigma_v, sigma_u;
    Param::TimeReal epoch_time;
    Param::TimeReal current_time;
};

#endif