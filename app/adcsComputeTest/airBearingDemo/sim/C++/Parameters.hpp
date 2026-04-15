#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include "Math.hpp"
#include <cmath>    // For mathematical constants and functions
#include <cstdint> // For fixed-width integer types
#include <ctime>    // For time-related functions
namespace Param {
    // Type definitions 
    // "Real" is a floating-point type used throughout the project
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
    
    enum class PointingMode { OFF, DETUMBLE, POINT };
    enum class TargetType { VECTOR, SUN, NADIR };

    struct MissionEvent {
        Real t_start;
        Real t_end;
        PointingMode mode;
        Vector3 face;
        TargetType target;
        Vector3 targetVec;
        Vector3 secondFace; // secondary face for secondary target 
        TargetType secondTarget; // secondary target type
        Vector3 targetVec2; // secondary target vector 
        Vector3 slewRates; 
    };

    // Simulation time parameters
    namespace SimTime {
        constexpr Real t_start = 0.0;          // Simulation start time
        constexpr Real t_end = 2000.0;          // Simulation end time
        constexpr Real Ts = 0.025;               // Time step size
        constexpr Real t_plot = 1.0;            // Data logging interval
        constexpr Real speed = 1.0;              // Simulation speed multiplier
        // std::chrono is heavy for embedded, we'll just store the epoch as 
        // a standard Unix timestamp (seconds since Jan 1, 1970)
       inline const Real epoch_timestamp = static_cast<Real>(std::time(nullptr)); // Current time as epoch
       //inline const Real epoch_timestamp = 1735689600.0; // Fixed timestamp for Jan 1, 2025
    }

    // Physical constants
    namespace Earth {
        constexpr Real G = 6.67408e-11;      // Gravitational constant in m^3/kg/s^2
        constexpr Real m_earth = 5.974e24;    // Earth's mass in kg
        constexpr Real mu_E = G * m_earth;    // Earth's gravitational parameter in m^3/s^2
        constexpr Real r_E = 6378137.0;     // Earth's radius in meters
        constexpr Real J2 = 1.08262668e-3;        // Earth's second zonal harmonic
        static const Vector3 omega_earth = Vector3{0.0, 0.0, 7.2921159e-5};
    }
    namespace Sun {
        constexpr Real S0 = 1361.0;          // Solar constant in W/m^2
        constexpr Real c = 299792458.0;      // Speed of light in m/s
        constexpr Real AU = 1.495978707e11; // Astronomical Unit in meters
    }
    
    namespace Spacecraft {
        constexpr Real mass = 5.0;           // Spacecraft mass in kg
        static const Vector3 I_principle = Vector3{0.0523, 0.0520, 0.0083}; // Principal moments of inertia in kg*m^2
        constexpr Real Ixy = 0.0032;              // Product of inertia Ixy in kg*m^2
        constexpr Real Ixz = -0.0018;             // Product of inertia Ixz in kg*m^2
        constexpr Real Iyz = 0.0025;              // Product of inertia Iyz in kg*m^2
        inline const Matrix3 I = []{
            Matrix3 m;
            m(0,0) = I_principle(0); m(0,1) = Ixy;           m(0,2) = Ixz;
            m(1,0) = Ixy;            m(1,1) = I_principle(1); m(1,2) = Iyz;
            m(2,0) = Ixz;            m(2,1) = Iyz;            m(2,2) = I_principle(2);
            return m;
        }(); // Inertia tensor in body frame
        static const Vector3 dim = Vector3{0.1, 0.1, 0.3}; // Spacecraft dimensions in meters
    }
    namespace Orbit {
        constexpr Real PI = 3.14159265358979323846;
        constexpr Real deg2rad = PI / 180.0;
        constexpr Real altitude = 600E3;        // Orbit altitude in meters
        constexpr Real inclination = 45.0 * deg2rad; // Orbit inclination in radians
        constexpr Real RAAN = 0.0 * deg2rad;               // Right Ascension of Ascending Node in radians
        constexpr Real argPeriapsis = 0.0 * deg2rad;        // Argument of periapsis in radians
        constexpr Real trueAnomaly = 0.0 * deg2rad;        // True anomaly at epoch in radians
        constexpr Real a = Earth::r_E + altitude; // Semi-major axis in meters
        constexpr Real e = 0.0;          // Orbit eccentricity

        // For initial state calculation
        struct FullState {
            Vector3 r_ECI; // Position vector in ECI frame in meters
            Vector3 v_ECI; // Velocity vector in ECI frame in meters per second
            Vector4 q0;   // Initial quaternion (identity)
            Vector3 omega; // Initial angular velocity (zero)
            Vector4 omega_wheel; // Initial wheel angular velocity (zero)
        };

        static const FullState InitialState = []{
            Real nu = trueAnomaly;
            Real i = inclination;
            Real O = RAAN;
            Real w = argPeriapsis;
            Real mu = Earth::mu_E;

            Real r_mag = a * (1.0 - e * e) / (1.0 + e * std::cos(nu));

            Vector3 r_PQW;
            r_PQW(0) = r_mag * std::cos(nu);
            r_PQW(1) = r_mag * std::sin(nu);
            r_PQW(2) = 0.0;
            Vector3 v_PQW;
            v_PQW(0) = -std::sqrt(mu / (a * (1.0 - e * e))) * std::sin(nu);
            v_PQW(1) = std::sqrt(mu / (a * (1.0 - e * e))) * (e + std::cos(nu));
            v_PQW(2) = 0.0;
            
            // RzO (Right Ascension)
            Matrix3 RzO;
            Real cO = std::cos(O), sO = std::sin(O);
            RzO(0,0) = cO;  RzO(0,1) = -sO; RzO(0,2) = 0.0;
            RzO(1,0) = sO;  RzO(1,1) = cO;  RzO(1,2) = 0.0;
            RzO(2,0) = 0.0; RzO(2,1) = 0.0; RzO(2,2) = 1.0;

            // Rxi (Inclination) - Rotation about X
            Matrix3 Rxi;
            Real ci = std::cos(i), si = std::sin(i);
            Rxi(0,0) = 1.0; Rxi(0,1) = 0.0; Rxi(0,2) = 0.0;
            Rxi(1,0) = 0.0; Rxi(1,1) = ci;  Rxi(1,2) = -si;
            Rxi(2,0) = 0.0; Rxi(2,1) = si;  Rxi(2,2) = ci;

            // Rzw (Argument of Periapsis)
            Matrix3 Rzw;
            Real cw = std::cos(w), sw = std::sin(w);
            Rzw(0,0) = cw;  Rzw(0,1) = -sw; Rzw(0,2) = 0.0;
            Rzw(1,0) = sw;  Rzw(1,1) = cw;  Rzw(1,2) = 0.0;
            Rzw(2,0) = 0.0; Rzw(2,1) = 0.0; Rzw(2,2) = 1.0;

            // Q = RzO * Rxi * Rzw;
            Matrix3 Q = RzO * Rxi * Rzw;

            // Final Calculation
            Vector3 r_ECI = Q * r_PQW;
            Vector3 v_ECI = Q * v_PQW;

            // Logic: [1; 0; 0; 0]
            Vector4 q0;
            q0(0) = 1.0; q0(1) = 0.0; q0(2) = 0.0; q0(3) = 0.0; 

            // omega: MATLAB overwrote the random value with zeros:
            // "omega = pi/180*[0;0;0];"
            //Vector3 w0 = Vector3(3, -3, 3) * Orbit::deg2rad; // Initial angular velocity in rad/s
            Vector3 w0 = Vector3::Zero();
            // omega_wheel: MATLAB "omega_wheel = 2*pi/60*[0;0;0;0];"
            Vector4 rw0 = Vector4::Zero();

            // Return the full package
            return FullState{r_ECI, v_ECI, q0, w0, rw0};
        }();
    }
    namespace Sensors {
        enum class ReductionMethod {
            IAU_2000_2006, 
            SIMPLE_ROTATION, 
            NONE
        };
        constexpr ReductionMethod reduction = ReductionMethod::IAU_2000_2006;
        // Accelerometers
        inline const Vector3 beta_a = Vector3{0.00, 0.00, 0.00}; // Accelerometer bias in m/s^2
        inline const Vector3 sigma_a = Vector3{0.0003, 0.0003, 0.0003}; // Accelerometer noise std dev in m/s^2
        // Gyros
        inline const Vector3 sigma_bias_walk = Vector3{0.002 * Orbit::deg2rad, 0.002 * Orbit::deg2rad, 0.002 * Orbit::deg2rad}; // Gyro bias walk std dev in rad/sqrt(s)
        inline const Vector3 beta_gyro = Vector3{0.02*Orbit::deg2rad, 0.02*Orbit::deg2rad, 0.02*Orbit::deg2rad}; // Gyro bias in rad/s
        inline const Vector3 sigma_gyro = Vector3{0.1*Orbit::deg2rad, 0.1*Orbit::deg2rad, 0.1*Orbit::deg2rad}; // Gyro noise std dev in rad/s
        // Magnetometers
        inline const Vector3 beta_mag = Vector3{2e-7, 2e-7, 2e-7}; // Magnetometer bias in Tesla
        inline const Vector3 sigma_mag = Vector3{5e-8, 5e-8, 5e-8}; // Magnetometer noise std dev in Tesla
        // Star Tracker 
        inline const Vector3 beta_star = Vector3{0.0, 0.0, 0.0}; // Star tracker bias in rad
        constexpr Real star_tracker_accuracy_arcsec = 100; // Star tracker accuracy in arcseconds
        constexpr Real sigma_star_rad = (100.0/3600.0)*(Orbit::PI/180.0); // Star tracker noise std dev in radians
        inline const Vector3 sigma_star = Vector3{sigma_star_rad, sigma_star_rad, sigma_star_rad}; // Star tracker noise std dev in radians
        constexpr Real star_update = 0.95; // Star tracker update rate in seconds
        inline const Vector4 q_bias = Vector4{1.0, 0.0, 0.0, 0.0}; // Star tracker quaternion bias
        constexpr Real small_angle_tol = 0.02; // Small angle tolerance in radians
        constexpr Real T_star = 0.5; // 1/hz, star tracker update period
        // Coarse Sun Sensors
        inline const Vector6 beta_css = Vector6{0.1e-3, 0.1e-3, 0.1e-3, 0.1e-3, 0.1e-3, 0.1e-3}; // CSS bias in A
        inline const Vector6 sigma_css = Vector6{0.2e-3, 0.2e-3, 0.2e-3, 0.2e-3, 0.2e-3, 0.2e-3}; // CSS noise std dev in A
        constexpr Real I_max = 10e-3; // Maximum CSS current in A
        // GPS
        constexpr Real T_gps = 0.2; // GPS update period in seconds
        constexpr Real k_gps = 1.0/1100.0; // 1/tau, pole of random walk
        inline const Vector3 beta_gps = Vector3{0.0, 0.0, 0.0}; // GPS position bias in meters
        inline const Vector3 sigma_gps_cep = Vector3{2.0, 2.0, 2.0}; // GPS CEP in meters
        constexpr Real K_gps = 1.0/1100.0; // GPS velocity random walk coefficient in 1/sqrt(s)
        constexpr Real sigma_cep = 1.699; // GPS std dev in meters
        inline const Vector3 sigma_V = Vector3{0.1, 0.1, 0.1}; // GPS velocity std dev in m/s
        // Reaction Wheels
        constexpr Real sigma_wheel = 1e-5; // Reaction wheel speed noise std dev in rad/s
        constexpr Real alpha_wheel = 0.01; // Reaction wheel friction coefficient
    }
    namespace Actuators {
        // Magnetorquers 
        constexpr Real m_max = 0.04; // Maximum magnetic moment in A*m^2
        constexpr Real m_min = -m_max; // Minimum magnetic moment in A*m^2
        constexpr Real k_desat = 25; // Desaturation gains
        
        // Reaction Wheels
        constexpr Real I_wheel = 1.13e-4; // Reaction wheel inertia in kg*m^2
        constexpr Real RPM_max = 13600.0; // Maximum reaction wheel speed in RPM
        constexpr Real RPM_min = -RPM_max; // Minimum reaction wheel speed in RPM
        constexpr Real omega_w_max = RPM_max * 2.0 * Orbit::PI / 60.0; // Maximum reaction wheel speed in rad/s
        constexpr Real omega_w_min = -omega_w_max; // Minimum reaction wheel speed in rad/s
        constexpr Real alpha_w_max = 90e3; // Maximum reaction wheel acceleration in RPM/s
        constexpr Real alpha_w_min = -alpha_w_max; // Minimum reaction wheel acceleration in RPM/s
        constexpr Real tau_w_max = 13e-3; // Maximum reaction wheel torque in N*m
        constexpr Real tau_w_min = -tau_w_max; // Minimum reaction wheel torque in N*m
        constexpr Real lambda_min_model = 0.1; // Geodesic smoothening requirement for reference
        constexpr Real k_null = 2e-7; // Null space control gain

        // Geometry 
        constexpr Real wheel_thickness = 0.0112; // Reaction wheel thickness in meters
        constexpr Real wheel_radius = 0.0162; // Reaction wheel radius in meters
        constexpr Real offset = 0.040; // Radial offset from center 
        constexpr Real theta = 50.0 * Orbit::deg2rad; // Angle from body axes in radians
        static const Matrix34 S = []{
            // MATLAB S matrix (3x4): each column is a wheel spin axis
            // S = [ cos(theta), -cos(theta),  0,            0;
            //       0,           0,           cos(theta),  -cos(theta);
            //       sin(theta),  sin(theta),  sin(theta),   sin(theta) ];
            Real theta = 50.0 * Orbit::deg2rad;
            Real c = std::cos(theta);
            Real s = std::sin(theta);

            Matrix34 m;
            // Explicitly set each element to avoid confusion about fill order
            // Column 0: [c, 0, s]
            m(0,0) = c;  m(1,0) = 0;  m(2,0) = s;
            // Column 1: [-c, 0, s]
            m(0,1) = -c; m(1,1) = 0;  m(2,1) = s;
            // Column 2: [0, c, s]
            m(0,2) = 0;  m(1,2) = c;  m(2,2) = s;
            // Column 3: [0, -c, s]
            m(0,3) = 0;  m(1,3) = -c; m(2,3) = s;

            // Normalize each column to unit length
            for(int i = 0; i < 4; ++i) {
                m.col(i).normalize();
            }

            return m;
        }(); // Reaction wheel orientation matrix
        static const Matrix43 S_pseudo = Math::pseudoInverse3x4(S); // Pseudoinverse of S
        inline const Matrix4 N = []{
            Matrix4 I4 = Matrix4::Identity();
            Matrix3 SS_t = S * S.transpose();
            Matrix3 invSSt = Math::inverse3x3(SS_t);
            Matrix43 St = S.transpose();
            return I4 - St * invSSt * S; // Reaction wheel null-space projector
        }();
        constexpr Real z_bias = -0.02; // Reaction wheel z-axis offset in meters
        static const Matrix34 r_center = []{
            // Calculate the radial component (Scale the S matrix)
            Matrix34 radial_part = S * offset;
            
            // Define the vertical offset vector
            Vector3 bias_vec;
            bias_vec(0) = 0.0; bias_vec(1) = 0.0; bias_vec(2) = z_bias;

            // Add the bias to EVERY column (This is the broadcasting part)
            // ".colwise()" tells Eigen to treat the matrix as a list of columns
            // Add the bias to EVERY column
            radial_part.colwiseAdd(bias_vec); 

            return radial_part;
        }();
        // 3. r_top (Center + Half Thickness * Direction)
        static const Matrix34 r_top = []{
            // Formula: Center + (0.5 * thickness * S)
            return r_center + (S * (0.5 * wheel_thickness));
        }();
        // 4. r_bottom (Center - Half Thickness * Direction)
        static const Matrix34 r_bottom = []{
            // Formula: Center - (0.5 * thickness * S)
            return r_center - (S * (0.5 * wheel_thickness));
        }();
        constexpr Real r_wheel = wheel_radius; // Reaction wheel radius
        constexpr Real t_wheel = wheel_thickness; // Reaction wheel thickness
        static const Vector4 I_rw = Vector4::Constant(I_wheel); // Reaction wheel inertia vector
    }
    namespace Controller {
        // NDI
        constexpr Real t_s_plant = 25;        // Plant settling time
        constexpr Real zeta_plant = 0.95;     // Damping ratio
        constexpr Real t_s_model = 45;       // Modeled system settling time
        constexpr Real zeta_model = 0.8;    // Modeled system damping ratio
        constexpr Real lambda_min_model = 0.1; // Minimum eigenvalue of modeled system
        // Bdot
        constexpr Real K_Bdot = 100000;      // Bdot gain
        constexpr Real alpha_BDot = 0.95;   // Bdot filter coefficient
        constexpr Real beta_fuse = 0.1;     // Low pass fuse coefficient
    }
    namespace Observer {
        inline const Matrix6 P_0 = []{
            Matrix6 m = Matrix6::Zero();
            Real angle_var = (Orbit::deg2rad*5.0)*(Orbit::deg2rad*5.0);
            Real rate_var = (Orbit::deg2rad*0.5)*(Orbit::deg2rad*0.5);
            m(0,0) = angle_var; m(1,1) = angle_var; m(2,2) = angle_var;
            m(3,3) = rate_var; m(4,4) = rate_var; m(5,5) = rate_var;
            return m;
        }(); // Position observer error covariance

        inline const Matrix6 G = []{
            Matrix6 m = Matrix6::Zero();
            m(0,0) = -1.0; m(1,1) = -1.0; m(2,2) = -1.0;
            m(3,3) = 1.0; m(4,4) = 1.0; m(5,5) = 1.0;
            return m;
        }(); // Position observer gain
        inline const Matrix6 Q = []{
            Matrix6 q_mat = Matrix6::Zero();
            // Top Left: Gyro Angle Random Walk (Density)
            Real gyro_density = Sensors::sigma_gyro(0) / std::sqrt(SimTime::Ts); // rad/sqrt(s)
            Real density_sq = gyro_density * gyro_density;

            // Bottom Right: Bias Random Walk
            Real bias_walk_sq = Sensors::sigma_bias_walk(0) * Sensors::sigma_bias_walk(0);

            q_mat(0,0) = density_sq; q_mat(1,1) = density_sq; q_mat(2,2) = density_sq;
            q_mat(3,3) = bias_walk_sq; q_mat(4,4) = bias_walk_sq; q_mat(5,5) = bias_walk_sq;
            return q_mat;
        }();
        inline const Matrix3 R_star = []{
            Matrix3 m = Matrix3::Zero();
            Real var = 4*Sensors::sigma_star_rad*Sensors::sigma_star_rad;
            m(0,0) = var; m(1,1) = var; m(2,2) = var;
            return m;
        }(); // Star tracker measurement noise covariance
        constexpr Real quest_accuracy = Orbit::deg2rad * 15;
        inline const Matrix3 R_quest = []{
            Matrix3 m = Matrix3::Zero();
            Real var = quest_accuracy*quest_accuracy;
            m(0,0) = var; m(1,1) = var; m(2,2) = var;
            return m;
        }(); // Quaternion measurement noise covariance
    }
}

#endif // PARAMETERS_HPP