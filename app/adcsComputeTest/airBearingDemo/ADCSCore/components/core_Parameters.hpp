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
        DETUMBLE, // BDot controller active
        POINT     // NDI controller active
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
        constexpr Real mass = static_cast<Real>(5.0);  // [kg]
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
            constexpr Real K_coil_x = static_cast<Real>(0.015);  // [m²] +X face coil constant (N*A)
            constexpr Real K_coil_nx = static_cast<Real>(0.015); // [m²] -X face coil constant
            constexpr Real K_coil_y = static_cast<Real>(0.015);  // [m²] +Y face coil constant
            constexpr Real K_coil_ny = static_cast<Real>(0.015); // [m²] -Y face coil constant
            constexpr Real I_max = static_cast<Real>(0.5);       // [A] Max per-coil current limit
            // Mapping: (m_x, m_y) → (I_Xpos, I_Xneg, I_Ypos, I_Yneg)
            // +X coil produces field in +X direction when I > 0
            // -X coil produces field in +X direction when I < 0 (polarity reversed)
            // Likewise for Y faces
        }

        // Reaction Wheels
        constexpr Real I_wheel = static_cast<Real>(1.13e-6);
        constexpr Real RPM_max = static_cast<Real>(12000.0);
        constexpr Real RPM_min = -RPM_max;
        constexpr Real omega_w_max = RPM_max * static_cast<Real>(2.0) * PI / static_cast<Real>(60.0);
        constexpr Real omega_w_min = -omega_w_max;
        constexpr Real tau_w_max = static_cast<Real>(13e-1);
        constexpr Real tau_w_min = -tau_w_max;
        constexpr Real k_null = static_cast<Real>(2e-7);

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

        // BEARING entry guard: only hand over to NDI once rates are calm.
        constexpr Real bearing_entry_axis_rate_threshold = static_cast<Real>(1.0) * deg2rad;
        
        // B-Dot gains (not used for air bearing, but kept for completeness)
        constexpr Real K_Bdot = static_cast<Real>(100000);
        constexpr Real alpha_BDot = static_cast<Real>(0.95);
        constexpr Real beta_fuse = static_cast<Real>(0.1);

        // Feed-forward compensation for wheel internal dynamics
        namespace FeedForward {
            constexpr bool enable_friction_comp = false;           // Compensate friction
            constexpr bool enable_ripple_comp = false;            // Compensate ripple (experimental)
            
            // Friction model parameters (match plant: Kt*I0/omega_nl)
            // Kt=8.3e-3, I0=0.1A, omega_nl=13600*2*pi/60=1424 rad/s
            constexpr Real b_viscous = static_cast<Real>(5.83e-7); // [N*m/(rad/s)] corrected
            constexpr Real tau_coulomb = static_cast<Real>(2.0e-5); // [N*m] Coulomb friction
            constexpr Real omega_eps = static_cast<Real>(0.1);    // [rad/s] Smoothing for sign
            
            // Ripple model parameters (match plant)
            constexpr Real ripple_amp = static_cast<Real>(2.0e-5); // [N*m] Ripple amplitude
            constexpr Real ripple_harmonic = static_cast<Real>(1.0); // Harmonic multiple
            
            // Feed-forward gain (tune for model mismatch robustness)
            constexpr Real ff_gain = static_cast<Real>(1.0);      // Scale factor [0, 1]
        }

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
            // Earth rotation (for frame transformations)
            constexpr Real omega_earth_z = static_cast<Real>(7.2921159e-5);

            // Reference directions for vector-based updates
            inline const Vector3 g_ref = Vector3{static_cast<Real>(0.0), static_cast<Real>(0.0), static_cast<Real>(1.0)};
            inline const Vector3 B_ref = Vector3{static_cast<Real>(1.0), static_cast<Real>(1.0), static_cast<Real>(1.0)}.normalized();

            // Measurement validity gates
            constexpr Real accel_min_norm = static_cast<Real>(0.1);
            constexpr Real mag_min_norm = static_cast<Real>(1e-9);

            // Initial gyro bias estimate [deg/s]
            inline const Vector3 beta_gyro_deg_s = Vector3{
                static_cast<Real>(0.02), static_cast<Real>(0.02), static_cast<Real>(0.02)};

            // Gyro white noise sigma [deg/s]
            inline const Vector3 sigma_gyro_deg_s = Vector3{
                static_cast<Real>(0.1), static_cast<Real>(0.1), static_cast<Real>(0.1)};

            // Gyro bias random walk sigma [deg/s]
            inline const Vector3 sigma_bias_walk_deg_s = Vector3{
                static_cast<Real>(0.001), static_cast<Real>(0.001), static_cast<Real>(0.001)};

            // Gyro bias correlation time [s]
            constexpr Real tau_bias = static_cast<Real>(3600.0);

            // Initial covariance sigmas
            constexpr Real p0_angle_sigma_deg = static_cast<Real>(8.0); // 5
            constexpr Real p0_bias_sigma_deg_s = static_cast<Real>(0.3); // 0.1

            // Star tracker/QUEST settings
            constexpr Real sigma_star_arcsec = static_cast<Real>(100.0); // 100
            constexpr Real quest_accuracy_deg = static_cast<Real>(15.0); // 15
            constexpr Real T_star = static_cast<Real>(0.5); // 0.5
            constexpr Real T_quest = static_cast<Real>(0.5); // 0.5

            // MEKF vector measurement covariance (unit-vector residual variance)
            constexpr Real R_accel_var = static_cast<Real>(5e-6); // 1e-4
            constexpr Real R_mag_var = static_cast<Real>(2e-4); // 1e-3

            // CSS parameter for sun vector synthesis
            constexpr Real I_max = static_cast<Real>(10e-3); // 10e-3
        }

        // ================================================================
        // DERIVED EXPORTS (consumed by observer/controller code)
        // ================================================================
        inline const Vector3 omega_earth = Vector3{static_cast<Real>(0.0), static_cast<Real>(0.0), Knobs::omega_earth_z};

        inline const Vector3 g_ref = Knobs::g_ref;
        inline const Vector3 B_ref = Knobs::B_ref;

        constexpr Real accel_min_norm = Knobs::accel_min_norm;
        constexpr Real mag_min_norm = Knobs::mag_min_norm;

        inline const Vector3 beta_gyro = Knobs::beta_gyro_deg_s * deg2rad;
        inline const Vector3 sigma_gyro = Knobs::sigma_gyro_deg_s * deg2rad;
        inline const Vector3 sigma_bias_walk = Knobs::sigma_bias_walk_deg_s * deg2rad;
        constexpr Real tau_bias = Knobs::tau_bias;

        constexpr Real sigma_star_rad = (Knobs::sigma_star_arcsec / static_cast<Real>(3600.0)) * deg2rad;
        constexpr Real quest_accuracy = Knobs::quest_accuracy_deg * deg2rad;
        constexpr Real T_star = Knobs::T_star;
        constexpr Real T_quest = Knobs::T_quest;
        constexpr Real I_max = Knobs::I_max;

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

        inline const Matrix3 R_star = [] {
            Matrix3 m = Matrix3::Zero();
            Real var = static_cast<Real>(4.0) * sigma_star_rad * sigma_star_rad;
            m(0,0) = var; m(1,1) = var; m(2,2) = var;
            return m;
        }();

        inline const Matrix3 R_quest = [] {
            Matrix3 m = Matrix3::Zero();
            Real var = quest_accuracy * quest_accuracy;
            m(0,0) = var; m(1,1) = var; m(2,2) = var;
            return m;
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