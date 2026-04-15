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
    using Vector11 = Math::Vec11;
    using Vector29 = Math::Vec29;
    using Matrix2 = Math::Mat2;
    using Matrix3 = Math::Mat3;
    using Matrix34 = Math::Mat34;
    using Matrix43 = Math::Mat43;
    using Matrix4 = Math::Mat4;
    using Matrix6 = Math::Mat6;

    // Math constants
    constexpr Real PI = static_cast<Real>(3.1415926535898);
    constexpr Real deg2rad = PI / static_cast<Real>(180.0);
    constexpr Real rad2deg = static_cast<Real>(180.0) / PI;

    // ========================================================================
    // SIMULATION TIMING
    // ========================================================================
    namespace SimTime {
        constexpr Real Ts = static_cast<Real>(0.025);  // Sample period [s] - 40 Hz loop rate
        constexpr Real t_start = static_cast<Real>(0.0);            // Simulation start [s]
        constexpr Real t_end = static_cast<Real>(120.0);           // Simulation end [s]
        constexpr Real t_plot = static_cast<Real>(0.1);             // Data logging interval [s]
        constexpr Real speed = static_cast<Real>(1.0);              // Simulation speed multiplier
        inline const TimeReal epoch_timestamp = static_cast<TimeReal>(std::time(nullptr));  // Use TimeReal for Unix timestamp precision
    }

    // In Plant_Parameters.hpp, replace/expand the Apparatus namespace:

    namespace Apparatus {
        // =====================================================================
        // AIR BEARING TEST BED GEOMETRY
        // =====================================================================
        // Reference: Pivot point is at center of air bearing sphere
        // Z-axis points UP (opposite to gravity)
        // 
        //         +Z (up)
        //          ^
        //          |    CubeSat
        //          |   +------+
        //          |   |      |  <-- h (CG offset from pivot)
        //          +---|  CG  |
        //       Pivot  |      |
        //          |   +------+
        //          |      |
        //          v   Bearing
        //         -Z (down)
        // =====================================================================
        
        // CG offset from pivot point [m]
        // Positive h = CG above pivot (unstable without control)
        // Negative h = CG below pivot (passively stable, pendulum-like)
        // |h| should be small (1-20mm) for challenging but achievable control
        constexpr Real h_cg = static_cast<Real>(0.000);  // [m] 5mm above pivot
        
        // Distance from pivot to base of cubesat [m]
        constexpr Real P = static_cast<Real>(0.1);
        
        // =====================================================================
        // DISTURBANCE PARAMETERS
        // =====================================================================
        // Disturbance force application point (measured from pivot) [m]
        constexpr Real r_disturbance = static_cast<Real>(0.15);  // Applied at side of cubesat
        
        // Disturbance force magnitude [N] - simulates finger push
        // ANALYSIS: Wheel torque capability ~17 mN·m per axis
        // Max rejectable disturbance: τ = F*r → F = 0.017/0.15 = 0.11 N
        // Using 0.08 N for ~20% margin: τ = 0.08*0.15 = 12 mN·m
        constexpr Real F_disturbance_max = static_cast<Real>(1.0);  // [N] gentle push
        
        // Disturbance timing
        constexpr Real T_disturbance_period = static_cast<Real>(15.0);  // [s] Period of periodic disturbance
        constexpr Real T_disturbance_duration = static_cast<Real>(0.5); // [s] Duration of each push
        
        // Disturbance direction in body frame (normalized)
        static const Vector3 disturbance_direction = Vector3{
            static_cast<Real>(1.0), static_cast<Real>(0.0), static_cast<Real>(0.0)
        };  // Push along +X body axis
        
        // =====================================================================
        // AIR BEARING FRICTION
        // =====================================================================
        // Viscous friction coefficient [Nm/(rad/s)]
        // Typical air bearings: 1e-6 to 1e-4 Nm·s/rad
        constexpr Real b_friction = static_cast<Real>(1e-5);
        
        // =====================================================================
        // HELMHOLTZ COILS (for magnetorquer testing)
        // =====================================================================
        // Magnetic field magnitude in lab frame [T]
        static const Vector3 B_helmholtz = Vector3{
            static_cast<Real>(30e-5),   // 30 µT in X
            static_cast<Real>(30e-5),     // 30 µT in Y  
            static_cast<Real>(30e-5)      // 30 µT in Z
        };
        
        // Legacy (can remove)
        static const Vector3 helmholtz_mag = B_helmholtz;
    }
    
    // =====================================================================
    // GRAVITY CONSTANT (for air bearing dynamics)
    // =====================================================================
    namespace Environment {
        constexpr Real g = static_cast<Real>(9.80665);  // [m/s^2]
        static const Vector3 g_inertial = Vector3{
            static_cast<Real>(0.0), 
            static_cast<Real>(0.0), 
            static_cast<Real>(-9.80665)  // Gravity points down (-Z in inertial)
        };
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
    }                               // Eccentricity

     struct FullState {
         Vector4 q0;
         Vector3 omega;
         Vector4 omega_wheel;
     };

    static const FullState InitialState = [] {
        Vector4 q0;
        q0(0) = static_cast<Real>(1.0); q0(1) = static_cast<Real>(0.0); q0(2) = static_cast<Real>(0.0); q0(3) = static_cast<Real>(0.0);
        Vector3 w0 = Vector3::Zero();
        Vector4 rw0 = Vector4::Zero();
        return FullState{q0, w0, rw0};
    }();

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
        constexpr Real omega_w_max = static_cast<Real>(7500.0) * static_cast<Real>(2.0) * PI / static_cast<Real>(60.0);
        constexpr Real omega_w_min = -omega_w_max;
        constexpr Real tau_w_max = static_cast<Real>(13e-1);
        constexpr Real tau_w_min = -tau_w_max;


        // Wheel Motor Electrical/Mechanical Specs 
        namespace WheelMotor {
            constexpr Real Kt = static_cast<Real>(8.3e-3);         // [N*m/A] Torque constant
            constexpr Real Ke = static_cast<Real>(8.3e-3);         // [V/(rad/s)] Back-EMF constant
            constexpr Real R = static_cast<Real>(3.95);            // [ohm] Phase-to-phase resistance
            constexpr Real L = static_cast<Real>(0.12e-3);         // [H] Phase-to-phase inductance
            constexpr Real tau_m = static_cast<Real>(0.0649);      // [s] Mechanical time constant
            constexpr Real tau_e = static_cast<Real>(0.03e-3);     // [s] Electrical time constant
            constexpr Real I0 = static_cast<Real>(0.1);            // [A] Typical no-load current
            constexpr Real no_load_rpm = static_cast<Real>(13600.0); // [rpm] No-load speed
            constexpr Real omega_no_load = no_load_rpm * static_cast<Real>(2.0) * PI / static_cast<Real>(60.0); // [rad/s]
        }

        // Wheel Internal Dynamics (unmodeled effects)
        namespace WheelDynamics {
            constexpr bool enable_motor_dynamics = true;           // First-order motor torque lag
            constexpr bool enable_friction = true;                 // Viscous + Coulomb friction
            constexpr bool enable_ripple = true;                   // Torque ripple / vibration

            constexpr Real motor_tau = WheelMotor::tau_m;          // [s] Motor torque time constant

            // Friction model: tau_fric = b*omega + tau_c*sign(omega)
            constexpr Real b_viscous = WheelMotor::Kt * WheelMotor::I0 / WheelMotor::omega_no_load; // [N*m/(rad/s)]
            constexpr Real tau_coulomb = static_cast<Real>(2.0e-5); // [N*m] Coulomb friction (tunable)
            constexpr Real omega_eps = static_cast<Real>(0.1);     // [rad/s] Smoothing for sign(omega)

            // Torque ripple model: tau_ripple = A*sin(h*theta)
            constexpr Real ripple_amp = static_cast<Real>(2.0e-5); // [N*m] Ripple amplitude (tunable)
            constexpr Real ripple_harmonic = static_cast<Real>(1.0); // [ ] Multiples of wheel spin rate
        }
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