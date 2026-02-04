#ifndef CORE_PARAMETERS_HPP
#define CORE_PARAMETERS_HPP

#include "core_Math.hpp"
#include "../Shared_Constants.hpp"
#include <cstdint>

namespace Param {
    
    // ========================================================================
    // TYPE DEFINITIONS
    // ========================================================================
    using Real = double;
    using Scalar = double;
    using Quat = Math::Vec4;
    using Vector3 = Math::Vec3;
    using Vector4 = Math::Vec4;
    using Vector6 = Math::Vec6;
    using Vector7 = Math::Vec7;
    using Vector17 = Math::Vec17;
    using Vector10 = Math::Vec10;
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

    // ========================================================================
    // TIMING
    // ========================================================================
    namespace SimTime {
        constexpr Real Ts = Shared::Ts;  // Sample period [s]
    }

    // ========================================================================
    // SPACECRAFT MODEL (Controller's estimate of truth)
    // ========================================================================
    namespace Spacecraft {
        constexpr Real mass = 5.0;  // [kg]
        static const Vector3 I_principle = Vector3{0.0523, 0.0520, 0.0083};
        constexpr Real Ixy = 0.0032;
        constexpr Real Ixz = -0.0018;
        constexpr Real Iyz = 0.0025;
        inline const Matrix3 I = [] {
            Matrix3 m;
            m(0,0) = I_principle(0); m(0,1) = Ixy;           m(0,2) = Ixz;
            m(1,0) = Ixy;            m(1,1) = I_principle(1); m(1,2) = Iyz;
            m(2,0) = Ixz;            m(2,1) = Iyz;            m(2,2) = I_principle(2);
            return m;
        }();
        static const Vector3 dim = Vector3{0.1, 0.1, 0.3};
    }

    // ========================================================================
    // ACTUATORS
    // ========================================================================
    namespace Actuators {
        // Magnetorquers
        constexpr Real m_max = 0.04;
        constexpr Real m_min = -m_max;
        constexpr Real k_desat = 25;

        // Reaction Wheels
        constexpr Real I_wheel = 1.13e-4;
        constexpr Real RPM_max = 13600.0;
        constexpr Real RPM_min = -RPM_max;
        constexpr Real omega_w_max = RPM_max * 2.0 * Shared::PI / 60.0;
        constexpr Real omega_w_min = -omega_w_max;
        constexpr Real tau_w_max = 13e-3;
        constexpr Real tau_w_min = -tau_w_max;
        constexpr Real k_null = 2e-7;

        // Wheel Geometry
        constexpr Real theta = 50.0 * Shared::deg2rad;
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
    // CONTROLLER GAINS
    // ========================================================================
    namespace Controller {
        constexpr Real t_s_plant = 25;
        constexpr Real zeta_plant = 0.95;
        constexpr Real t_s_model = 45;
        constexpr Real zeta_model = 0.8;
        constexpr Real lambda_min_model = 0.1;
        constexpr Real K_Bdot = 100000;
        constexpr Real alpha_BDot = 0.95;
        constexpr Real beta_fuse = 0.1;
    }

    // ========================================================================
    // OBSERVER TUNING
    // ========================================================================
    namespace Observer {
        // Earth rotation (for frame transformations)
        static const Vector3 omega_earth = Vector3{0.0, 0.0, 7.2921159e-5};
        
        // Gyro parameters (vector forms for initialization)
        static const Vector3 beta_gyro = Vector3{
            0.02 * Shared::deg2rad, 0.02 * Shared::deg2rad, 0.02 * Shared::deg2rad};
        static const Vector3 sigma_gyro = Vector3{
            0.1 * Shared::deg2rad, 0.1 * Shared::deg2rad, 0.1 * Shared::deg2rad};
        static const Vector3 sigma_bias_walk = Vector3{
            0.002 * Shared::deg2rad, 0.002 * Shared::deg2rad, 0.002 * Shared::deg2rad};
        
        // Scalar versions for covariance matrices
        constexpr Real sigma_gyro_rad = 0.1 * Shared::deg2rad;
        constexpr Real sigma_bias_walk_rad = 0.002 * Shared::deg2rad;
        constexpr Real sigma_star_rad = (100.0 / 3600.0) * Shared::deg2rad;

        // Update periods
        constexpr Real T_star = 0.5;   // Star tracker update period [s]
        constexpr Real T_quest = 0.5;  // QUEST update period [s]
        
        // CSS parameter for sun vector synthesis
        constexpr Real I_max = 10e-3;  // Maximum CSS current [A]

        inline const Matrix6 P_0 = [] {
            Matrix6 m = Matrix6::Zero();
            Real angle_var = (5.0 * Shared::deg2rad) * (5.0 * Shared::deg2rad);
            Real rate_var = (0.5 * Shared::deg2rad) * (0.5 * Shared::deg2rad);
            m(0,0) = angle_var; m(1,1) = angle_var; m(2,2) = angle_var;
            m(3,3) = rate_var;  m(4,4) = rate_var;  m(5,5) = rate_var;
            return m;
        }();

        inline const Matrix6 G = [] {
            Matrix6 m = Matrix6::Zero();
            m(0,0) = -1.0; m(1,1) = -1.0; m(2,2) = -1.0;
            m(3,3) = 1.0;  m(4,4) = 1.0;  m(5,5) = 1.0;
            return m;
        }();

        inline const Matrix6 Q = [] {
            Matrix6 q_mat = Matrix6::Zero();
            Real gyro_density = sigma_gyro_rad / std::sqrt(SimTime::Ts);
            Real density_sq = gyro_density * gyro_density;
            Real bias_walk_sq = sigma_bias_walk_rad * sigma_bias_walk_rad;
            q_mat(0,0) = density_sq; q_mat(1,1) = density_sq; q_mat(2,2) = density_sq;
            q_mat(3,3) = bias_walk_sq; q_mat(4,4) = bias_walk_sq; q_mat(5,5) = bias_walk_sq;
            return q_mat;
        }();

        inline const Matrix3 R_star = [] {
            Matrix3 m = Matrix3::Zero();
            Real var = 4 * sigma_star_rad * sigma_star_rad;
            m(0,0) = var; m(1,1) = var; m(2,2) = var;
            return m;
        }();

        constexpr Real quest_accuracy = 15.0 * Shared::deg2rad;
        inline const Matrix3 R_quest = [] {
            Matrix3 m = Matrix3::Zero();
            Real var = quest_accuracy * quest_accuracy;
            m(0,0) = var; m(1,1) = var; m(2,2) = var;
            return m;
        }();
    }
}

#endif // CORE_PARAMETERS_HPP