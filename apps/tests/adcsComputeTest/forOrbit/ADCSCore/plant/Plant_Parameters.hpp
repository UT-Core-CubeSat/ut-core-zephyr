#ifndef PLANT_PARAMETERS_HPP
#define PLANT_PARAMETERS_HPP

#include "../components/core_Math.hpp"
using namespace Math;
namespace PlantParam {
    // ========================================================================
    // TYPE DEFINITIONS (mirror core for compatibility)
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
    using Vector29 = Math::Vec29;
    using Matrix2 = Math::Mat2;
    using Matrix3 = Math::Mat3;
    using Matrix34 = Math::Mat34;
    using Matrix43 = Math::Mat43;
    using Matrix4 = Math::Mat4;
    using Matrix6 = Math::Mat6;

    // State Machine 
    enum class MissionMode { 
    SAFE,      // Safe mode, totally off
    DETUMBLE,       // Detumble, minimal actuator use
    STANDBY,    // Sun-pointing for power generation
    DOWNLINK,   // Point antenna face at ground station
    IMAGING,    // Point boresight at earth target (nadir or specific)
    CUSTOM      // Escape hatch for explicit vector control
    };

    struct GroundTarget {
    double latitude;   // [deg] geodetic
    double longitude;  // [deg]
    double altitude;   // [m] above WGS84 ellipsoid (0 for surface)
    };

    struct Command {
        MissionMode mode;
        
        // Used for DOWNLINK and IMAGING modes
        GroundTarget target;
        
        // For IMAGING: true = point at target, false = point nadir
        bool track_target;
        
        // For CUSTOM mode only (escape hatch)
        Math::Vec<3> body_axis;
        Math::Vec<3> target_eci;
    };
    // Math constants
    constexpr Real PI = static_cast<Real>(3.1415926535898);
    constexpr Real deg2rad = PI / static_cast<Real>(180.0);
    constexpr Real rad2deg = static_cast<Real>(180.0) / PI;

    // ========================================================================
    // SIMULATION TIMING
    // ========================================================================
    namespace SimTime {
        constexpr Real Ts = static_cast<Real>(0.025);  // Sample period [s] - 40 Hz loop rate;          // Sample period [s] (from shared)
        constexpr Real t_start = static_cast<Real>(0.0);            // Simulation start [s]
        constexpr Real t_end = static_cast<Real>(2000.0);           // Simulation end [s]
        constexpr Real t_plot = static_cast<Real>(1.0);             // Data logging interval [s]
        constexpr Real speed = static_cast<Real>(1.0);              // Simulation speed multiplier
        inline const TimeReal epoch_timestamp = static_cast<TimeReal>(std::time(nullptr));  // Use TimeReal for Unix timestamp precision
    }

    // ========================================================================
    // ENVIRONMENT MODELS
    // ========================================================================
    namespace Earth {
        constexpr Real G = static_cast<Real>(6.67408e-11);          // [m^3/kg/s^2] Gravitational constant
        constexpr Real m_earth = static_cast<Real>(5.974e24);       // [kg] Earth mass
        constexpr Real mu_E = G * m_earth;       // [m^3/s^2] Gravitational parameter
        constexpr Real r_E = static_cast<Real>(6378137.0);          // [m] Earth radius
        constexpr Real J2 = static_cast<Real>(1.08262668e-3);       // J2 perturbation
        static const Vector3 omega_earth = Vector3{static_cast<Real>(0.0), static_cast<Real>(0.0), static_cast<Real>(7.2921159e-5)};
    }

    namespace Sun {
        constexpr Real S0 = static_cast<Real>(1361.0);              // [W/m^2] Solar constant
        constexpr Real c = static_cast<Real>(299792458.0);          // [m/s] Speed of light
        constexpr Real AU = static_cast<Real>(1.495978707e11);      // [m] Astronomical Unit
    }

    // ========================================================================
    // SPACECRAFT TRUTH (what the plant actually simulates)
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
    // ORBIT & INITIAL CONDITIONS
    // ========================================================================
    namespace Orbit {
        constexpr Real altitude = static_cast<Real>(600E3);                        // [m]
        constexpr Real inclination = static_cast<Real>(45.0 * deg2rad);    // [rad]
        constexpr Real RAAN = static_cast<Real>(0.0 * deg2rad);
        constexpr Real argPeriapsis = static_cast<Real>(0.0 * deg2rad);
        constexpr Real trueAnomaly = static_cast<Real>(0.0 * deg2rad);
        constexpr Real a = Earth::r_E + altitude;               // [m] Semi-major axis
        constexpr Real e = static_cast<Real>(0.0);                                 // Eccentricity

        struct FullState {
            Vector3 r_ECI;
            Vector3 v_ECI;
            Vector4 q0;
            Vector3 omega;
            Vector4 omega_wheel;
        };

        static const FullState InitialState = [] {
            Real nu = static_cast<Real>(trueAnomaly);
            Real i = static_cast<Real>(inclination);
            Real O = static_cast<Real>(RAAN);
            Real w = static_cast<Real>(argPeriapsis);
            Real mu = Earth::mu_E;

            Real r_mag = a * (static_cast<Real>(1.0) - e * e) / (static_cast<Real>(1.0) + e * std::cos(nu));

            Vector3 r_PQW;
            r_PQW(0) = r_mag * std::cos(nu);
            r_PQW(1) = r_mag * std::sin(nu);
            r_PQW(2) = static_cast<Real>(0.0);

            Vector3 v_PQW;
            v_PQW(0) = -std::sqrt(mu / (a * (static_cast<Real>(1.0) - e * e))) * std::sin(nu);
            v_PQW(1) = std::sqrt(mu / (a * (static_cast<Real>(1.0) - e * e))) * (e + std::cos(nu));
            v_PQW(2) = static_cast<Real>(0.0);

            // Rotation matrices
            Real cO = std::cos(O), sO = std::sin(O);
            Real ci = std::cos(i), si = std::sin(i);
            Real cw = std::cos(w), sw = std::sin(w);

            Matrix3 RzO, Rxi, Rzw;
            RzO(0,0) = cO;  RzO(0,1) = -sO; RzO(0,2) = static_cast<Real>(0.0);
            RzO(1,0) = sO;  RzO(1,1) = cO;  RzO(1,2) = static_cast<Real>(0.0);
            RzO(2,0) = static_cast<Real>(0.0); RzO(2,1) = static_cast<Real>(0.0); RzO(2,2) = static_cast<Real>(1.0);

            Rxi(0,0) = static_cast<Real>(1.0); Rxi(0,1) = static_cast<Real>(0.0); Rxi(0,2) = static_cast<Real>(0.0);
            Rxi(1,0) = static_cast<Real>(0.0); Rxi(1,1) = ci;  Rxi(1,2) = -si;
            Rxi(2,0) = static_cast<Real>(0.0); Rxi(2,1) = si;  Rxi(2,2) = ci;

            Rzw(0,0) = cw;  Rzw(0,1) = -sw; Rzw(0,2) = static_cast<Real>(0.0);
            Rzw(1,0) = sw;  Rzw(1,1) = cw;  Rzw(1,2) = static_cast<Real>(0.0);
            Rzw(2,0) = static_cast<Real>(0.0); Rzw(2,1) = static_cast<Real>(0.0); Rzw(2,2) = static_cast<Real>(1.0);

            Matrix3 Q = RzO * Rxi * Rzw;
            Vector3 r_ECI = Q * r_PQW;
            Vector3 v_ECI = Q * v_PQW;

            Vector4 q0;
            q0(0) = static_cast<Real>(1.0); q0(1) = static_cast<Real>(0.0); q0(2) = static_cast<Real>(0.0); q0(3) = static_cast<Real>(0.0);
            Vector3 w0 = Vector3::Zero();
            Vector4 rw0 = Vector4::Zero();

            return FullState{r_ECI, v_ECI, q0, w0, rw0};
        }();
    }

    // ========================================================================
    // SENSOR MODELS (Truth: biases and noise for measurement generation)
    // ========================================================================
    namespace Sensors {
        enum class ReductionMethod { IAU_2000_2006, SIMPLE_ROTATION, NONE };
        constexpr ReductionMethod reduction = ReductionMethod::IAU_2000_2006;

        // Accelerometer
        inline const Vector3 beta_a = Vector3{static_cast<Real>(0.0), static_cast<Real>(0.0), static_cast<Real>(0.0)};
        inline const Vector3 sigma_a = Vector3{static_cast<Real>(0.0003), static_cast<Real>(0.0003), static_cast<Real>(0.0003)};

        // Gyroscope
        inline const Vector3 sigma_bias_walk = Vector3{
            static_cast<Real>(0.002) * deg2rad, static_cast<Real>(0.002) * deg2rad, static_cast<Real>(0.002) * deg2rad};
        inline const Vector3 beta_gyro = Vector3{
            static_cast<Real>(0.02) * deg2rad, static_cast<Real>(0.02) * deg2rad, static_cast<Real>(0.02) * deg2rad};
        inline const Vector3 sigma_gyro = Vector3{
            static_cast<Real>(0.1) * deg2rad, static_cast<Real>(0.1) * deg2rad, static_cast<Real>(0.1) * deg2rad};       
        // Magnetometer
        inline const Vector3 beta_mag = Vector3{static_cast<Real>(2e-7), static_cast<Real>(2e-7), static_cast<Real>(2e-7)};
        inline const Vector3 sigma_mag = Vector3{static_cast<Real>(5e-8), static_cast<Real>(5e-8), static_cast<Real>(5e-8)};

        // Star Tracker
        inline const Vector3 beta_star = Vector3{static_cast<Real>(0.0), static_cast<Real>(0.0), static_cast<Real>(0.0)};
        constexpr Real sigma_star_rad = (static_cast<Real>(100.0) / static_cast<Real>(3600.0)) * deg2rad;
        inline const Vector3 sigma_star = Vector3{sigma_star_rad, sigma_star_rad, sigma_star_rad};
        constexpr Real T_star = static_cast<Real>(0.5);  // [s] Update period
        constexpr Real small_angle_tol = static_cast<Real>(0.02);
        inline const Vector4 q_bias = Vector4{static_cast<Real>(1.0), static_cast<Real>(0.0), static_cast<Real>(0.0), static_cast<Real>(0.0)};
        constexpr Real star_update = static_cast<Real>(0.95);  // Legacy name, same as T_star

        // Coarse Sun Sensors
        inline const Vector6 beta_css = Vector6{static_cast<Real>(0.1e-3), static_cast<Real>(0.1e-3), static_cast<Real>(0.1e-3), static_cast<Real>(0.1e-3), static_cast<Real>(0.1e-3), static_cast<Real>(0.1e-3)};
        inline const Vector6 sigma_css = Vector6{static_cast<Real>(0.2e-3), static_cast<Real>(0.2e-3), static_cast<Real>(0.2e-3), static_cast<Real>(0.2e-3), static_cast<Real>(0.2e-3), static_cast<Real>(0.2e-3)};
        constexpr Real I_max = static_cast<Real>(10e-3);

        // GPS
        constexpr Real T_gps = static_cast<Real>(0.2);
        constexpr Real k_gps = static_cast<Real>(1.0) / static_cast<Real>(1100.0);
        constexpr Real K_gps = static_cast<Real>(1.0)/static_cast<Real>(1100.0); // GPS velocity random walk coefficient in 1/sqrt(s)
        inline const Vector3 beta_gps = Vector3{static_cast<Real>(0.0), static_cast<Real>(0.0), static_cast<Real>(0.0)};
        inline const Vector3 sigma_gps_cep = Vector3{static_cast<Real>(2.0), static_cast<Real>(2.0), static_cast<Real>(2.0)};
        constexpr Real sigma_cep = static_cast<Real>(1.699);
        inline const Vector3 sigma_V = Vector3{static_cast<Real>(0.1), static_cast<Real>(0.1), static_cast<Real>(0.1)};

        // Reaction Wheel Encoders
        constexpr Real sigma_wheel = static_cast<Real>(1e-5);
        constexpr Real alpha_wheel = static_cast<Real>(0.01);
    }

    // ========================================================================
    // ACTUATOR TRUTH (what the plant actually simulates)
    // ========================================================================
    namespace Actuators {
        // Magnetorquers
        constexpr Real m_max = static_cast<Real>(0.04);
        constexpr Real m_min = -m_max;

        // Reaction Wheels
        constexpr Real I_wheel = static_cast<Real>(1.13e-6);
        constexpr Real omega_w_max = static_cast<Real>(13600.0) * static_cast<Real>(2.0) * PI / static_cast<Real>(60.0);
        constexpr Real omega_w_min = -omega_w_max;
        constexpr Real tau_w_max = static_cast<Real>(13e-3);
        constexpr Real tau_w_min = -tau_w_max;

        // Wheel Geometry (truth)
        constexpr Real wheel_thickness = static_cast<Real>(0.0112);
        constexpr Real wheel_radius = static_cast<Real>(0.0162);
        constexpr Real offset = static_cast<Real>(0.040);
        constexpr Real z_bias = static_cast<Real>(-0.02);
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

        static const Vector4 I_rw = Vector4::Constant(I_wheel);
    }
}

#endif // PLANT_PARAMETERS_HPP