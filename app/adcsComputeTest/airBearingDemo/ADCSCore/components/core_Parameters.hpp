#ifndef CORE_PARAMETERS_HPP
#define CORE_PARAMETERS_HPP

#include "core_Math.hpp"
using namespace Math;
namespace Param {
    
    // TYPE DEFINITIONS
    using Real = Math::Real;
    using TimeReal = Math::TimeReal;
    using Scalar = Real;
    using Quat = Math::Vec4;
    using Vector3 = Math::Vec3;
    using Vector4 = Math::Vec4;
    using Vector6 = Math::Vec6;
    using Vector7 = Math::Vec7;
    using Vector17 = Math::Vec17;
    using Vector10 = Math::Vec10;
    using Vector11 = Math::Vec11;
    using Vector13 = Math::Vec13;
    using Vector29 = Math::Vec29;
    using Matrix2 = Math::Mat2;
    using Matrix3 = Math::Mat3;
    using Matrix34 = Math::Mat34;
    using Matrix43 = Math::Mat43;
    using Matrix36 = Math::Mat36;
    using Matrix63 = Math::Mat63;
    using Matrix4 = Math::Mat4;
    using Matrix6 = Math::Mat6;

    // INTERNAL CONTROL MODES
    // These are what the controller understands as the current mode of operation.
    // The Core class will map these to the PointingMode based on sensor data and state.
    enum class PointingMode {
        OFF,      // No control
        DETUMBLE, // BDot (MTQ only, wheels idle)
        MOTOR,    // NDI (wheels only, no MTQ desat)
        BOTH      // NDI + MTQ desaturation
    };

    namespace Apparatus {
        // Controllers estimate 
        constexpr Real h_cg = static_cast<Real>(-0.005); // [m] CG offset from air bearing pivot
    }


    // SPACECRAFT CONFIGURATION
    // Body frame axis definitions for each function

    namespace Config {
        // Which body axis corresponds to each function
        // !! Edit these to match the actual physical configuration of the spacecraft !!
        static const Vector3 face_solar = Vector3{0, 1, 0};     // +Y: solar panels
        static const Vector3 face_antenna = Vector3{1, 0, 0};   // +X: S-band antenna
        static const Vector3 face_boresight = Vector3{0, 0, -1}; // -Z: camera/payload
        static const Vector3 face_star_tracker = Vector3{0, -1, 0}; // -Y: star tracker
    }

    // Math constants
    constexpr Real PI = Math::PI;
    constexpr Real g = static_cast<Real>(9.80665); // [m/s^2] standard gravity
    constexpr Real deg2rad = PI / static_cast<Real>(180.0);
    constexpr Real rad2deg = static_cast<Real>(180.0) / PI;


    // SPACECRAFT MODEL (Controller's estimate of truth)

    namespace Spacecraft {
        constexpr Real mass = static_cast<Real>(3.0);  // [kg]
        static const Vector3 I_principle = Vector3{static_cast<Real>(0.0523), static_cast<Real>(0.0520), static_cast<Real>(0.0083)};
        constexpr Real Ixy = static_cast<Real>(0.0032);
        constexpr Real Ixz = static_cast<Real>(-0.0018);
        constexpr Real Iyz = static_cast<Real>(0.0025);
        inline const Matrix3 I = [] {
            Matrix3 m;
            m(0,0) = I_principle(0); m(0,1) = Ixy;           m(0,2) = Ixz;
            m(1,0) = Ixy;            m(1,1) = I_principle(1); m(1,2) = Iyz;
            m(2,0) = Ixz;            m(2,1) = Iyz;            m(2,2) = I_principle(2);
            return m;
        }();
        static const Vector3 dim = Vector3{static_cast<Real>(0.1), static_cast<Real>(0.1), static_cast<Real>(0.3)};
    }


    // ACTUATORS

    namespace Actuators {
        // Magnetorquers: Body Dipole Command [A·m²]
        constexpr Real m_max = static_cast<Real>(0.04);  // [A·m²]
        constexpr Real m_min = -m_max;
        constexpr Real k_desat = static_cast<Real>(15);

        // Coil Calibration: Per-Face Magnetorquer PCB Design
        // 4 embedded coils on long faces: +X, -X, +Y, -Y (Z faces inactive)
        // K_coil[i] = N_turns * A_coil [m²] for face i
        // Used to convert body dipole → per-face current: I_face = m_cmd / K_coil
        // PLACEHOLDER: Calibrate with actual PCB measurements or Helmholtz test
        namespace Coils {
            constexpr Real K_coil_x = static_cast<Real>(0.34);  // [m²] +X face coil constant (N*A)
            constexpr Real K_coil_nx = static_cast<Real>(0.34); // [m²] -X face coil constant
            constexpr Real K_coil_y = static_cast<Real>(0.34);  // [m²] +Y face coil constant
            constexpr Real K_coil_ny = static_cast<Real>(0.34); // [m²] -Y face coil constant
            constexpr Real I_max = static_cast<Real>(0.15);       // [A] Max per-coil current limit
            // Mapping: (m_x, m_y) → (I_Xpos, I_Xneg, I_Ypos, I_Yneg)
            // +X coil produces field in +X direction when I > 0
            // -X coil produces field in +X direction when I < 0 (polarity reversed)
            // Likewise for Y faces
        }

        // Reaction Wheels
        constexpr Real I_wheel = static_cast<Real>(1.13e-4);
        constexpr Real RPM_max = static_cast<Real>(12000.0);
        constexpr Real RPM_min = -RPM_max;
        constexpr Real omega_w_max = RPM_max * static_cast<Real>(2.0) * PI / static_cast<Real>(60.0);
        constexpr Real omega_w_min = -omega_w_max;
        // Max wheel torque [N*m]. 13 mN*m matches motor capability order of magnitude.
        constexpr Real tau_w_max = static_cast<Real>(13e-1);
        constexpr Real tau_w_min = -tau_w_max;
        constexpr Real k_null = static_cast<Real>(2e-7);

        // Back-EMF reversal protection: minimum fraction of tau_w_max allowed when
        // the commanded torque opposes wheel spin direction.
        // At zero speed the full torque is allowed; at omega_w_max only this fraction.
        constexpr Real tau_rev_min_frac = static_cast<Real>(1.0);

        // Deadband target: hold wheels near ±omega_deadband to keep hall sensors active.
        // For the current wheel geometry this corresponds to [+,+,-,-] up to global sign.
        constexpr Real omega_deadband = static_cast<Real>(0.0) * static_cast<Real>(2.0) * PI / static_cast<Real>(60.0); // [rad/s]
        // Keep deadband bias loop slow relative to motor dynamics to avoid chatter/limit cycles.
        constexpr Real k_deadband = static_cast<Real>(8e-8); // [N·m·s/rad] 8e-5

        // Wheel Geometry
        constexpr Real theta = static_cast<Real>(50.0) * deg2rad;
        static const Matrix34 S = [] {
            Real c = std::cos(theta);
            Real s = std::sin(theta);
            Matrix34 m;
            m(0,0) = c;  m(1,0) = 0;  m(2,0) = s;
            m(0,1) = -c; m(1,1) = 0;  m(2,1) = s;
            m(0,2) = 0;  m(1,2) = c;  m(2,2) = s;
            m(0,3) = 0;  m(1,3) = -c; m(2,3) = s;
            for (int i = 0; i < 4; ++i) {
                Real n = std::sqrt(m(0,i)*m(0,i) + m(1,i)*m(1,i) + m(2,i)*m(2,i));
                if (n > 1e-15f) { m(0,i) /= n; m(1,i) /= n; m(2,i) /= n; }
            }
            return m;
        }();
        static const Matrix43 S_pseudo = Math::pseudoInverse3x4(S);
        inline const Matrix4 N = [] {
            Matrix4 I4 = Matrix4::Identity();
            Matrix3 SS_t = S * S.transpose();
            Matrix3 invSSt = Math::inverse3x3(SS_t);
            Matrix43 St = S.transpose();
            return I4 - St * invSSt * S;
        }();
        static const Vector4 I_rw = Vector4::Constant(I_wheel);
    }

    // CONTROLLER GAINS (Tuned for Air Bearing Demo)

    namespace Controller {
        constexpr Real t_s_plant = static_cast<Real>(4);
        constexpr Real zeta_plant = static_cast<Real>(0.9);
        constexpr Real t_s_model = static_cast<Real>(6);
        constexpr Real zeta_model = static_cast<Real>(0.85);
        constexpr Real lambda_min_model = static_cast<Real>(0.1);

        // B-Dot gains (not used for air bearing, but kept for completeness)
        constexpr Real K_Bdot = static_cast<Real>(100000);
        constexpr Real alpha_BDot = static_cast<Real>(0.98);
        constexpr Real beta_fuse = static_cast<Real>(0.1);

        // Priority MTQ desaturation scheduler for BEARING mode
        namespace Desat {
            constexpr Real entry_rate_norm = static_cast<Real>(5.0) * deg2rad;        // [rad/s]
            constexpr Real enter_wheel_speed_ratio = static_cast<Real>(0.90);          // [ ] of omega_w_max
            constexpr Real exit_wheel_speed_ratio = static_cast<Real>(0.70);           // [ ] of omega_w_max
            constexpr Real enter_momentum_norm = static_cast<Real>(8e-4);              // [N*m*s]
            constexpr Real exit_momentum_norm = static_cast<Real>(3e-4);               // [N*m*s]
            constexpr Real enter_hold_time = static_cast<Real>(0.5);                   // [s]
            constexpr Real exit_hold_time = static_cast<Real>(2.0);                    // [s]
            constexpr Real filter_tau = static_cast<Real>(25.0);                       // [s]
            constexpr Real mtq_dipole_scale = static_cast<Real>(0.35);                 // [ ] slow unloading
            constexpr Real wheel_attitude_scale = static_cast<Real>(0.35);             // [ ] soften wheel fight
            constexpr Real wheel_mtq_comp_scale = static_cast<Real>(0.15);             // [ ] partial MTQ cancellation
        }
    }


    // OBSERVER TUNING

    namespace Observer {
        // ================================================================
        // TUNABLE KNOBS (edit this block first)
        // ================================================================
        namespace Knobs {
            // Reference directions for vector-based updates
            inline const Vector3 g_ref = Vector3{static_cast<Real>(0.0), static_cast<Real>(0.0), static_cast<Real>(1.0)};
            inline const Vector3 B_ref = Vector3{static_cast<Real>(1.0), static_cast<Real>(0.0), static_cast<Real>(0.0)}.normalized();

            // Magnetometer fusion master switch. Disable while bench-testing
            // outside a Helmholtz cage — indoor field distortion causes yaw to
            // drift to a different "rest" after any large board motion.
            // Re-enable in the cage (or once a trusted B_ref is available).
            constexpr bool use_magnetometer = true;

            // Measurement validity gates
            // accel: expect ~9.8 m/s^2 at rest; reject obviously bad reads.
            // mag:   driver outputs Tesla; Earth field is ~25-65 uT; reject < 15 uT.
            constexpr Real accel_min_norm = static_cast<Real>(5.0);
            constexpr Real mag_min_norm = static_cast<Real>(1.0e-5);  // bench: mag fusion gives ~18 uT

            // Fallback gyro bias [deg/s] used only during the observer's pre-init window;
            // the TRIAD self-init in ObserverClass overrides this with the current
            // session's measured static mean (~2 s average).
            inline const Vector3 beta_gyro_deg_s = Vector3{
                static_cast<Real>(-0.02744), static_cast<Real>(-0.21728), static_cast<Real>(-0.24858)};

            // Gyro white noise sigma [deg/s]  (measured static std, sensor_capture_20260422)
            inline const Vector3 sigma_gyro_deg_s = Vector3{
                static_cast<Real>(0.0672), static_cast<Real>(0.079139), static_cast<Real>(0.068928)};

            // Gyro bias random walk sigma [deg/s]
            // Must be >0 — otherwise P_bias decays to zero (via F's Gauss-Markov term)
            // and the filter stops tracking bias drift, causing slow attitude drift.
            inline const Vector3 sigma_bias_walk_deg_s = Vector3{
                static_cast<Real>(0.003), static_cast<Real>(0.003), static_cast<Real>(0.003)};

            // Gyro bias correlation time [s]
            // Shorter = filter adapts bias faster to observed attitude residuals.
            constexpr Real tau_bias = static_cast<Real>(600.0);

            // Initial covariance sigmas (post-TRIAD: attitude ~2deg, bias ~0.05 deg/s)
            constexpr Real p0_angle_sigma_deg = static_cast<Real>(2.0); // was 8.0
            constexpr Real p0_bias_sigma_deg_s = static_cast<Real>(0.05); // was 0.3

            // MEKF vector measurement covariance (unit-vector residual variance).
            // R_accel: accel sees gravity + sensor noise; tight = strong tilt anchor.
            // R_mag: Helmholtz cage field is clean and B_ref is exact, so we can trust it.
            //   Simulation value 1e-5 ≈ 0.18° yaw uncertainty per update — keeps the filter
            //   tracking yaw actively without yaw drift from the integrated Z-gyro bias.
            //   For real indoor use, raise back to 1e-3 to tolerate lab field distortion.
            constexpr Real R_accel_var = static_cast<Real>(2e-6);
            constexpr Real R_mag_var = static_cast<Real>(1e-5);
        }

        // ================================================================
        // DERIVED EXPORTS (consumed by observer/controller code)
        // ================================================================
        inline const Vector3 g_ref = Knobs::g_ref;
        inline const Vector3 B_ref = Knobs::B_ref;

        constexpr Real accel_min_norm = Knobs::accel_min_norm;
        constexpr Real mag_min_norm = Knobs::mag_min_norm;
        constexpr bool use_magnetometer = Knobs::use_magnetometer;

        inline const Vector3 beta_gyro = Knobs::beta_gyro_deg_s * deg2rad;
        inline const Vector3 sigma_gyro = Knobs::sigma_gyro_deg_s * deg2rad;
        inline const Vector3 sigma_bias_walk = Knobs::sigma_bias_walk_deg_s * deg2rad;
        constexpr Real tau_bias = Knobs::tau_bias;

        inline const Matrix6 P_0 = [] {
            Matrix6 m = Matrix6::Zero();
            Real angle_var = (Knobs::p0_angle_sigma_deg * deg2rad) * (Knobs::p0_angle_sigma_deg * deg2rad);
            Real bias_var = (Knobs::p0_bias_sigma_deg_s * deg2rad) * (Knobs::p0_bias_sigma_deg_s * deg2rad);
            m(0,0) = angle_var; m(1,1) = angle_var; m(2,2) = angle_var;
            m(3,3) = bias_var;  m(4,4) = bias_var;  m(5,5) = bias_var;
            return m;
        }();

        inline const Matrix6 G = [] {
            Matrix6 m = Matrix6::Zero();
            m(0,0) = static_cast<Real>(-1.0); m(1,1) = static_cast<Real>(-1.0); m(2,2) = static_cast<Real>(-1.0);
            m(3,3) = static_cast<Real>(1.0);  m(4,4) = static_cast<Real>(1.0);  m(5,5) = static_cast<Real>(1.0);
            return m;
        }();

        // Discrete-time process noise template (actual filter still scales by dt)
        inline const Matrix6 Q = [] {
            Matrix6 q_mat = Matrix6::Zero();
            q_mat(0,0) = sigma_gyro(0) * sigma_gyro(0);
            q_mat(1,1) = sigma_gyro(1) * sigma_gyro(1);
            q_mat(2,2) = sigma_gyro(2) * sigma_gyro(2);
            q_mat(3,3) = sigma_bias_walk(0) * sigma_bias_walk(0);
            q_mat(4,4) = sigma_bias_walk(1) * sigma_bias_walk(1);
            q_mat(5,5) = sigma_bias_walk(2) * sigma_bias_walk(2);
            return q_mat;
        }();

        inline const Matrix3 R_accel = [] {
            Matrix3 m = Matrix3::Identity() * Knobs::R_accel_var;
            return m;
        }();

        inline const Matrix3 R_mag = [] {
            Matrix3 m = Matrix3::Identity() * Knobs::R_mag_var;
            return m;
        }();
    }
}

#endif // CORE_PARAMETERS_HPP