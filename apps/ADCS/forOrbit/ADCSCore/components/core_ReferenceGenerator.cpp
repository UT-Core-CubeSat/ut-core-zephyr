#include "core_ReferenceGenerator.hpp"
#include "ADCSCore.hpp"  // For ADCS::Command, etc.

// ============================================================================
// CONSTRUCTOR
// ============================================================================
ReferenceGenerator::ReferenceGenerator() 
    : helpers_(),
      last_q_ref_({1, 0, 0, 0}),
      initialized_(false),
      last_q_ff_({1, 0, 0, 0}),
      last_mode_(ADCS::MissionMode::SAFE),
      feedforward_initialized_(false)
{
}

// ============================================================================
// MAIN UPDATE
// ============================================================================
ReferenceGenerator::RefOutput ReferenceGenerator::update(
    const StateVector& states, 
    const ADCS::Command& command,
    TimeReal unix_time, Scalar dt) 
{
    // Reset feedforward BEFORE mode handlers if mode changed
    // This prevents using stale quaternion data from previous mode
    if (command.mode != last_mode_) {
        feedforward_initialized_ = false;
        last_mode_ = command.mode;
    }

    RefGenOutput gen;
    Param::PointingMode internal_mode;

    // Dispatch based on commanded mission mode
    switch (command.mode) {
        case ADCS::MissionMode::SAFE:
            gen = handleSafe(states, dt);
            internal_mode = Param::PointingMode::OFF;
            break;

        case ADCS::MissionMode::DETUMBLE:
            gen = handleDetumble(states, dt);
            internal_mode = Param::PointingMode::DETUMBLE;
            break;

        case ADCS::MissionMode::STANDBY:
            gen = handleStandby(states, unix_time, dt);
            internal_mode = Param::PointingMode::POINT;
            break;

        case ADCS::MissionMode::DOWNLINK:
            gen = handleDownlink(states, command.target, unix_time, dt);
            internal_mode = Param::PointingMode::POINT;
            break;

        case ADCS::MissionMode::IMAGING:
            gen = handleImaging(states, command.target, command.track_target, unix_time, dt);
            internal_mode = Param::PointingMode::POINT;
            break;

        case ADCS::MissionMode::CUSTOM:
            gen = handleCustom(states, 
                              Vector3{command.body_axis(0), command.body_axis(1), command.body_axis(2)},
                              Vector3{command.target_eci(0), command.target_eci(1), command.target_eci(2)},
                              unix_time, dt);
            internal_mode = Param::PointingMode::POINT;
            break;

        default:
            gen = handleSafe(states, dt);
            internal_mode = Param::PointingMode::OFF;
            break;
    }

    // Post-processing: enforce quaternion continuity
    gen.q_ref = enforceQuaternionContinuity(gen.q_ref);
    last_q_ref_ = gen.q_ref;

    // Pack output
    Reference out;
    out.setSegment(0, gen.q_ref);      // indices 0-3
    out.setSegment(4, gen.omega_ref);  // indices 4-6
    out.setSegment(7, gen.alpha_ref);  // indices 7-9

    return RefOutput{out, internal_mode};
}

// ============================================================================
// MODE HANDLERS
// ============================================================================

ReferenceGenerator::RefGenOutput ReferenceGenerator::handleDetumble(const StateVector& states, Scalar dt) {
    // DETUMBLE mode: Just hold current attitude, let BDot controller handle detumble
    RefGenOutput output;
    output.q_ref = states.segment<4>(6);  // Current attitude
    output.omega_ref = Vector3::Zero();    // Zero rate target
    output.alpha_ref = Vector3::Zero();
    return output;
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::handleSafe(const StateVector& states, Scalar dt) {
    // SAFE mode: Just hold current attitude, reference won't matter
    RefGenOutput output;
    output.q_ref = states.segment<4>(6);  // Current attitude
    output.omega_ref = Vector3::Zero();    // Zero rate target
    output.alpha_ref = Vector3::Zero();
    return output;
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::handleStandby(
    const StateVector& states, 
    TimeReal unix_time, Scalar dt) 
{
    // STANDBY: Point solar panels (+Y) at Sun
    // Secondary: Keep star tracker (-Y) away from Sun (redundant with primary)
    //            Or: Point +X toward velocity for thermal balance
    
    PointingObjective primary;
    primary.body_axis = Param::Config::face_solar;  // +Y (solar panels)
    primary.target_eci = getSunVector(unix_time);
    primary.weight = static_cast<Scalar>(1.0);

    PointingObjective secondary;
    secondary.body_axis = Param::Config::face_boresight;  // -Z (camera/boresight)
    secondary.target_eci = getNadirVector(states);  // Point camera at Earth (useful)
    secondary.weight = static_cast<Scalar>(0.5);

    return computeAttitude(primary, secondary, states, dt);
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::handleDownlink(
    const StateVector& states, 
    const ADCS::GroundTarget& target,
    TimeReal unix_time, Scalar dt) 
{
    // DOWNLINK: Point antenna (+X) at ground station
    // Secondary: Maximize solar power (+Y toward Sun)
    
    PointingObjective primary;
    primary.body_axis = Param::Config::face_antenna;  // +X (S-band antenna)
    primary.target_eci = getGroundTargetVector(states, target, unix_time);
    primary.weight = static_cast<Scalar>(1.0);

    PointingObjective secondary;
    secondary.body_axis = Param::Config::face_solar;  // +Y (solar panels)
    secondary.target_eci = getSunVector(unix_time);
    secondary.weight = static_cast<Scalar>(0.5);

    return computeAttitude(primary, secondary, states, dt);
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::handleImaging(
    const StateVector& states,
    const ADCS::GroundTarget& target,
    bool track_target,
    TimeReal unix_time, Scalar dt)
{
    // IMAGING: Point boresight (-Z) at target or nadir
    // Secondary: Maximize solar power (+Y toward Sun)
    
    PointingObjective primary;
    primary.body_axis = Param::Config::face_boresight;  // -Z (camera)
    
    if (track_target) {
        primary.target_eci = getGroundTargetVector(states, target, unix_time);
    } else {
        primary.target_eci = getNadirVector(states);
    }
    primary.weight = static_cast<Scalar>(1.0);

    PointingObjective secondary;
    secondary.body_axis = Param::Config::face_solar;  // +Y (solar panels)
    secondary.target_eci = getSunVector(unix_time);
    secondary.weight = static_cast<Scalar>(0.5);

    return computeAttitude(primary, secondary, states, dt);
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::handleCustom(
    const StateVector& states,
    const Vector3& body_axis,
    const Vector3& target_eci,
    TimeReal unix_time, Scalar dt)
{
    // CUSTOM: Direct control, still use Sun as secondary
    PointingObjective primary;
    primary.body_axis = body_axis.normalized();
    primary.target_eci = target_eci.normalized();
    primary.weight = static_cast<Scalar>(1.0);

    PointingObjective secondary;
    secondary.body_axis = Param::Config::face_solar;
    secondary.target_eci = getSunVector(unix_time);
    secondary.weight = static_cast<Scalar>(0.5);

    return computeAttitude(primary, secondary, states, dt);
}

// ============================================================================
// CORE POINTING ALGORITHM
// ============================================================================

ReferenceGenerator::RefGenOutput ReferenceGenerator::computeAttitude(
    const PointingObjective& primary,
    const PointingObjective& secondary,
    const StateVector& states, Scalar dt)
{
    // Use the two-vector pointing algorithm
    // Primary axis is satisfied exactly, secondary is best-effort
    
    Vector3 b1 = primary.body_axis.normalized();
    Vector3 t1 = primary.target_eci.normalized();
    Vector3 b2 = secondary.body_axis.normalized();
    Vector3 t2 = secondary.target_eci.normalized();

    RefGenOutput output;
    output.q_ref = helpers_.quatFromTwoVectorPairs(b1, t1, b2, t2);
    output.omega_ref = computeFeedforwardOmega(output.q_ref, dt);
    output.alpha_ref = Vector3::Zero();
    
    return output;
}

// ============================================================================
// TARGET RESOLUTION
// ============================================================================

ReferenceGenerator::Vector3 ReferenceGenerator::getSunVector(TimeReal unix_time) {
    TimeReal jd = helpers_.julianDate(unix_time);
    return helpers_.earth2sun(jd).normalized();
}

ReferenceGenerator::Vector3 ReferenceGenerator::getNadirVector(const StateVector& states) {
    Vector3 R = states.segment<3>(0);  // Position in ECI
    return (-R).normalized();  // Nadir points toward Earth center
}

ReferenceGenerator::Vector3 ReferenceGenerator::getAntiSunVector(TimeReal unix_time) {
    return -getSunVector(unix_time);
}

ReferenceGenerator::Vector3 ReferenceGenerator::getGroundTargetVector(
    const StateVector& states,
    const ADCS::GroundTarget& target,
    TimeReal unix_time)
{
    // Convert ground target LLA to ECI
    // 1. LLA -> ECEF
    // 2. ECEF -> ECI (using current time)
    
    Scalar lat_rad = target.latitude * Param::deg2rad;
    Scalar lon_rad = target.longitude * Param::deg2rad;
    Scalar alt = target.altitude;
    
    // WGS84 ellipsoid parameters
    constexpr Scalar a = static_cast<Scalar>(6378137.0);           // Semi-major axis [m]
    constexpr Scalar f = static_cast<Scalar>(1.0) / static_cast<Scalar>(298.257223563); // Flattening
    constexpr Scalar e2 = static_cast<Scalar>(2)*f - f*f;          // Eccentricity squared
    
    // Prime vertical radius of curvature
    Scalar sin_lat = std::sin(lat_rad);
    Scalar cos_lat = std::cos(lat_rad);
    Scalar N = a / std::sqrt(static_cast<Scalar>(1.0) - e2 * sin_lat * sin_lat);
    
    // ECEF position
    Vector3 r_ecef;
    r_ecef(0) = (N + alt) * cos_lat * std::cos(lon_rad);
    r_ecef(1) = (N + alt) * cos_lat * std::sin(lon_rad);
    r_ecef(2) = (N * (static_cast<Scalar>(1.0) - e2) + alt) * sin_lat;
    
    // ECEF -> ECI rotation
    TimeReal jd = helpers_.julianDate(unix_time);
    Param::Matrix3 R_ecef2eci = helpers_.dcmeci2ecef(jd).transpose();
    
    Vector3 r_target_eci = R_ecef2eci * r_ecef;
    
    // Vector from spacecraft to target
    Vector3 r_sc = states.segment<3>(0);  // Spacecraft position in ECI
    Vector3 los = r_target_eci - r_sc;    // Line of sight
    
    return los.normalized();
}

// ============================================================================
// UTILITIES
// ============================================================================

ReferenceGenerator::Quat ReferenceGenerator::enforceQuaternionContinuity(const Quat& q_new) {
    if (!initialized_) {
        initialized_ = true;
        return q_new;
    }
    if (last_q_ref_.dot(q_new) < 0) {
        return -q_new;
    }
    return q_new;
}

ReferenceGenerator::Vector3 ReferenceGenerator::computeFeedforwardOmega(const Quat& q_ref, Scalar dt) {
    Vector3 omega_ref = Vector3::Zero();

    // Protect against dt = 0 (first update or very small timestep)
    if (dt < static_cast<Scalar>(1e-9)) {
        last_q_ff_ = q_ref;
        feedforward_initialized_ = true;
        return omega_ref;  // Return zero omega
    }

    if (!feedforward_initialized_) {
        last_q_ff_ = q_ref;
        feedforward_initialized_ = true;
        // Return zero omega on first call (no derivative possible)
    } else {
        // Numerical differentiation: q_dot ≈ (q_new - q_old) / dt
        Quat q_dot = (q_ref - last_q_ff_) / dt;
        
        // ω = 2 * q* ⊗ q_dot (vector part)
        Quat q_conj = helpers_.quatconj(last_q_ff_);
        Quat omega_quat = helpers_.quatMultiply(q_conj, q_dot);
        omega_ref = static_cast<Scalar>(2.0) * omega_quat.tail<3>();
        
        // Sanity check: clamp unreasonable angular rates (> 1 rad/s)
        constexpr Scalar max_omega = static_cast<Scalar>(1.0);  // rad/s
        for (int i = 0; i < 3; ++i) {
            if (omega_ref(i) > max_omega) omega_ref(i) = max_omega;
            if (omega_ref(i) < -max_omega) omega_ref(i) = -max_omega;
        }
        
        last_q_ff_ = q_ref;
    }
    
    return omega_ref;
}