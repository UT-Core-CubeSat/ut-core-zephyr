#ifndef CORE_PARAMETERS_HPP
#define CORE_PARAMETERS_HPP

#include "core_Math.hpp"
using namespace Math;
namespace Param {
    
    // ========================================================================
    // TYPE DEFINITIONS
    // ========================================================================
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

    // ========================================================================
    // INTERNAL CONTROL MODES
    // These are what the controller understands (simpler than MissionMode)
    // ========================================================================
    enum class PointingMode { 
        OFF,      // No control
        DETUMBLE, // BDot controller active
        POINT     // NDI controller active
    };

    namespace Apparatus {
        // Controllers estimate 
        constexpr Real h_cg = static_cast<Real>(0.005); // [m] CG offset from air bearing pivot
    }

    // ========================================================================
    // SPACECRAFT CONFIGURATION
    // Body frame axis definitions for each function
    // ========================================================================
    namespace Config {
        // Which body axis corresponds to each function
        // Edit these to match your spacecraft design
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

    // ========================================================================
    // SPACECRAFT MODEL (Controller's estimate of truth)
    // ========================================================================
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

    // ========================================================================
    // ACTUATORS
    // ========================================================================
    namespace Actuators {
        // Magnetorquers (disabled for air bearing - no varying B-field)
        constexpr Real m_max = static_cast<Real>(0.04);  // [A·m²]
        constexpr Real m_min = -m_max;
        constexpr Real k_desat = static_cast<Real>(15);

        // Reaction Wheels
        constexpr Real I_wheel = static_cast<Real>(1.13e-6);
        constexpr Real RPM_max = static_cast<Real>(7500.0);
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
            for (int i = 0; i < 4; ++i) m.col(i).normalize();
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

    // ========================================================================
    // CONTROLLER GAINS (Tuned for Air Bearing Demo)
    // ========================================================================
    namespace Controller {
        constexpr Real t_s_plant = static_cast<Real>(4);
        constexpr Real zeta_plant = static_cast<Real>(0.9);
        constexpr Real t_s_model = static_cast<Real>(6);
        constexpr Real zeta_model = static_cast<Real>(0.85);
        constexpr Real lambda_min_model = static_cast<Real>(0.1);
        
        // B-Dot gains (not used for air bearing, but kept for completeness)
        constexpr Real K_Bdot = static_cast<Real>(100000);
        constexpr Real alpha_BDot = static_cast<Real>(0.95);
        constexpr Real beta_fuse = static_cast<Real>(0.1);

        // Feed-forward compensation for wheel internal dynamics
        namespace FeedForward {
            constexpr bool enable_friction_comp = true;           // Compensate friction
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
    }

    // ========================================================================
    // OBSERVER TUNING
    // ========================================================================
    namespace Observer {
        // Earth rotation (for frame transformations)
        static const Vector3 omega_earth = Vector3{static_cast<Real>(0.0), static_cast<Real>(0.0), static_cast<Real>(7.2921159e-5)};
        
        // Gyro parameters (vector forms for initialization)
        static const Vector3 beta_gyro = Vector3{
            static_cast<Real>(0.02) * deg2rad, static_cast<Real>(0.02) * deg2rad, static_cast<Real>(0.02) * deg2rad};
        static const Vector3 sigma_gyro = Vector3{
            static_cast<Real>(0.1) * deg2rad, static_cast<Real>(0.1) * deg2rad, static_cast<Real>(0.1) * deg2rad};
        static const Vector3 sigma_bias_walk = Vector3{
            static_cast<Real>(0.002) * deg2rad, static_cast<Real>(0.002) * deg2rad, static_cast<Real>(0.002) * deg2rad};
        
        // Scalar versions for covariance matrices
        constexpr Real sigma_gyro_rad = static_cast<Real>(0.1) * deg2rad;
        constexpr Real sigma_bias_walk_rad = static_cast<Real>(0.002) * deg2rad;
        constexpr Real sigma_star_rad = (static_cast<Real>(100.0) / static_cast<Real>(3600.0)) * deg2rad;

        // Update periods
        constexpr Real T_star = static_cast<Real>(0.5);   // Star tracker update period [s]
        constexpr Real T_quest = static_cast<Real>(0.5);  // QUEST update period [s]
        
        // CSS parameter for sun vector synthesis
        constexpr Real I_max = static_cast<Real>(10e-3);  // Maximum CSS current [A]

        inline const Matrix6 P_0 = [] {
            Matrix6 m = Matrix6::Zero();
            Real angle_var = (static_cast<Real>(5.0) * deg2rad) * (static_cast<Real>(5.0) * deg2rad);
            Real bias_var = (static_cast<Real>(0.1) * deg2rad) * (static_cast<Real>(0.1) * deg2rad);  // 0.1 deg/s bias uncertainty
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

        // Discrete-time process noise (applied per timestep, will be scaled by dt in observer)
        inline const Matrix6 Q = [] {
            Matrix6 q_mat = Matrix6::Zero();
            // Gyro noise variance per sample: sigma^2
            Real gyro_var = sigma_gyro_rad * sigma_gyro_rad;
            // Bias random walk variance per sample: sigma^2
            Real bias_walk_var = sigma_bias_walk_rad * sigma_bias_walk_rad;
            q_mat(0,0) = gyro_var; q_mat(1,1) = gyro_var; q_mat(2,2) = gyro_var;
            q_mat(3,3) = bias_walk_var; q_mat(4,4) = bias_walk_var; q_mat(5,5) = bias_walk_var;
            return q_mat;
        }();

        inline const Matrix3 R_star = [] {
            Matrix3 m = Matrix3::Zero();
            Real var = static_cast<Real>(4) * sigma_star_rad * sigma_star_rad;
            m(0,0) = var; m(1,1) = var; m(2,2) = var;
            return m;
        }();

        constexpr Real quest_accuracy = static_cast<Real>(15.0) * deg2rad;
        inline const Matrix3 R_quest = [] {
            Matrix3 m = Matrix3::Zero();
            Real var = quest_accuracy * quest_accuracy;
            m(0,0) = var; m(1,1) = var; m(2,2) = var;
            return m;
        }();
    }
}

#endif // CORE_PARAMETERS_HPP