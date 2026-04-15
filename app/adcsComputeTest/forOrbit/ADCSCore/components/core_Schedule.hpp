#ifndef CORE_SCHEDULE_HPP
#define CORE_SCHEDULE_HPP

#include "core_Parameters.hpp"
#include <vector>
namespace Mission {

    // ============================================================
    // MISSION SCHEDULE CONFIGURATION
    // Edit this file to modify mission events
    // Orbital period @ 600km ≈ 5760 seconds (~96 minutes)
    // ============================================================

    // Helper to create a normalized Vec3
    inline Param::Vector3 makeNormalized(Param::Scalar x, Param::Scalar y, Param::Scalar z) {
        Param::Vector3 v;
        v(0) = x; v(1) = y; v(2) = z;
        return v.normalized();
    }

    // Helper to create a Vec3
    inline Param::Vector3 makeVec3(Param::Scalar x, Param::Scalar y, Param::Scalar z) {
        Param::Vector3 v;
        v(0) = x; v(1) = y; v(2) = z;
        return v;
    }

    inline std::vector<Param::MissionEvent> getSchedule() {
        return {
            
            // ========================================================
            // ORBIT 1: Post-Deployment Stabilization & Initial Power
            // ========================================================
            
            // [0] DETUMBLE: Initial stabilization after deployment
            // Duration: 10 minutes - let B-dot controller kill tip-off rates
            Param::MissionEvent{
                0.0,                                                // t_start
                60.0,                                              // t_end
                Param::PointingMode::DETUMBLE,                      // mode
                Param::Vector3::UnitZ(),                            // face (unused in detumble)
                Param::TargetType::VECTOR,                          // target (unused)
                Param::Vector3::Zero(),                             // targetVec (unused)
                Param::Vector3::UnitX(),                            // secondFace (unused)
                Param::TargetType::VECTOR,                          // secondTarget (unused)
                Param::Vector3::Zero(),                             // targetVec2 (unused)
                Param::Vector3::Zero()                              // slewRates
            },
            
            // [1] SUN POINTING: +Z to Sun, +X toward Nadir (power generation)
            // Duration: 50 minutes - primary solar charging mode
            Param::MissionEvent{
                60.0,                                              // t_start
                120.0,                                             // t_end
                Param::PointingMode::POINT,                         // mode
                Param::Vector3::UnitZ(),                            // face: +Z axis
                Param::TargetType::SUN,                             // target: Sun
                Param::Vector3::Zero(),                             // targetVec (unused for SUN)
                Param::Vector3::UnitX(),                            // secondFace: +X axis
                Param::TargetType::NADIR,                           // secondTarget: toward Earth
                Param::Vector3::Zero(),                             // targetVec2 (unused for NADIR)
                Param::Vector3::Zero()                              // slewRates
            },
            
            // [2] NADIR POINTING: +Z to Nadir, +X toward Sun (Earth observation)
            // Duration: 36 minutes - imaging pass
            Param::MissionEvent{
                120.0,                                             // t_start
                180.0,                                             // t_end (end of orbit 1)
                Param::PointingMode::POINT,                         // mode
                Param::Vector3::UnitZ(),                            // face: +Z axis (camera)
                Param::TargetType::NADIR,                           // target: Earth center
                Param::Vector3::Zero(),                             // targetVec (unused)
                Param::Vector3::UnitX(),                            // secondFace: +X axis
                Param::TargetType::SUN,                             // secondTarget: panels toward sun
                Param::Vector3::Zero(),                             // targetVec2 (unused)
                Param::Vector3::Zero()                              // slewRates
            },
            
            // ========================================================
            // ORBIT 2: Mixed Operations
            // ========================================================
            
            // [3] INERTIAL HOLD: Fixed pointing for star tracker calibration
            // Point +Z at Polaris (approximately), +X toward +X ECI
            // Duration: 20 minutes
            Param::MissionEvent{
                180.0,                                             // t_start
                300.0,                                             // t_end
                Param::PointingMode::POINT,                         // mode
                Param::Vector3::UnitZ(),                            // face: +Z axis
                Param::TargetType::VECTOR,                          // target: fixed inertial
                makeVec3(0.0, 0.0, 1.0),                            // targetVec: +Z ECI (near Polaris)
                Param::Vector3::UnitX(),                            // secondFace: +X axis
                Param::TargetType::VECTOR,                          // secondTarget: fixed inertial
                makeVec3(1.0, 0.0, 0.0),                            // targetVec2: +X ECI
                Param::Vector3::Zero()                              // slewRates
            },
            
            // [4] SUN POINTING: -Z to Sun (test opposite axis), +Y toward Nadir
            // Duration: 30 minutes - test flexibility
            Param::MissionEvent{
                300.0,                                             // t_start
                480.0,                                             // t_end
                Param::PointingMode::POINT,                         // mode
                -Param::Vector3::UnitZ(),                           // face: -Z axis (flip!)
                Param::TargetType::SUN,                             // target: Sun
                Param::Vector3::Zero(),                             // targetVec (unused)
                Param::Vector3::UnitY(),                            // secondFace: +Y axis
                Param::TargetType::NADIR,                           // secondTarget: toward Earth
                Param::Vector3::Zero(),                             // targetVec2 (unused for NADIR)
                Param::Vector3::Zero()                              // slewRates
            },
            
            // [5] LARGE SLEW TEST: Nadir with +Z to Sun (big reorientation)
            // Duration: 46 minutes - tests large-angle maneuver + tracking
            Param::MissionEvent{
                480.0,                                             // t_start
                690.0,                                            // t_end (end of orbit 2)
                Param::PointingMode::POINT,                         // mode
                Param::Vector3::UnitX(),                            // face: +X axis (90° from previous)
                Param::TargetType::NADIR,                           // target: Nadir
                Param::Vector3::Zero(),                             // targetVec (unused)
                Param::Vector3::UnitZ(),                            // secondFace: +Z axis
                Param::TargetType::SUN,                             // secondTarget: Sun
                Param::Vector3::Zero(),                             // targetVec2 (unused)
                Param::Vector3::Zero()                              // slewRates
            },
            
            // ========================================================
            // ORBIT 3: Edge Cases & Stress Tests
            // ========================================================
            
            // [6] ANTI-SUN POINTING: +Z away from Sun (tests near-180° geometry)
            // Secondary: +X toward Nadir
            // Duration: 25 minutes
            Param::MissionEvent{
                690.0,                                            // t_start
                900.0,                                            // t_end
                Param::PointingMode::POINT,                         // mode
                Param::Vector3::UnitZ(),                            // face: +Z axis
                Param::TargetType::VECTOR,                          // target: Anti-sun direction
                makeVec3(-1.0, 0.0, 0.0),                           // targetVec: Placeholder
                Param::Vector3::UnitX(),                            // secondFace: +X axis
                Param::TargetType::NADIR,                           // secondTarget: Nadir
                Param::Vector3::Zero(),                             // targetVec2 (unused)
                Param::Vector3::Zero()                              // slewRates
            },
            
            // [7] DIAGONAL AXIS TEST: Non-principal axis pointing
            // Point body diagonal toward Sun
            // Duration: 30 minutes
            Param::MissionEvent{
                900.0,                                            // t_start
                1080.0,                                            // t_end
                Param::PointingMode::POINT,                         // mode
                makeNormalized(1.0, 1.0, 1.0),                      // face: body diagonal
                Param::TargetType::SUN,                             // target: Sun
                Param::Vector3::Zero(),                             // targetVec (unused)
                makeNormalized(1.0, -1.0, 0.0),                     // secondFace: perpendicular
                Param::TargetType::NADIR,                           // secondTarget: Nadir
                Param::Vector3::Zero(),                             // targetVec2 (unused)
                Param::Vector3::Zero()                              // slewRates
            },
            
            // [8] RAPID MODE SWITCH: Quick detumble re-test
            // Simulates anomaly recovery
            // Duration: 5 minutes
            Param::MissionEvent{
                1080.0,                                            // t_start
                1380.0,                                            // t_end
                Param::PointingMode::DETUMBLE,                      // mode
                Param::Vector3::UnitZ(),                            // face (unused)
                Param::TargetType::VECTOR,                          // target (unused)
                Param::Vector3::Zero(),                             // targetVec (unused)
                Param::Vector3::UnitX(),                            // secondFace (unused)
                Param::TargetType::VECTOR,                          // secondTarget (unused)
                Param::Vector3::Zero(),                             // targetVec2 (unused)
                Param::Vector3::Zero()                              // slewRates
            },
            
            // [9] RECOVERY TO SUN: Post-anomaly safe mode
            // Duration: remaining orbit 3 (~36 minutes)
            Param::MissionEvent{
                1380.0,                                            // t_start
                1500.0,                                            // t_end (end of orbit 3)
                Param::PointingMode::POINT,                         // mode
                Param::Vector3::UnitZ(),                            // face: +Z axis
                Param::TargetType::SUN,                             // target: Sun
                Param::Vector3::Zero(),                             // targetVec (unused)
                Param::Vector3::UnitX(),                            // secondFace: +X axis
                Param::TargetType::NADIR,                           // secondTarget: Nadir
                Param::Vector3::Zero(),                             // targetVec2 (unused)
                Param::Vector3::Zero()                              // slewRates
            },
            
            // ========================================================
            // ORBIT 4: Sustained Operations & Final Tests
            // ========================================================
            
            // [10] GROUND TARGET TRACK: Simulated ground station pass
            // Point +Z at Nadir (approx ground track)
            // Duration: 15 minutes (realistic pass duration)
            Param::MissionEvent{
                1500.0,                                            // t_start
                1590.0,                                            // t_end
                Param::PointingMode::POINT,                         // mode
                Param::Vector3::UnitZ(),                            // face: +Z axis (antenna/camera)
                Param::TargetType::NADIR,                           // target: Nadir (approx ground track)
                Param::Vector3::Zero(),                             // targetVec (unused)
                Param::Vector3::UnitY(),                            // secondFace: +Y axis
                Param::TargetType::SUN,                             // secondTarget: power
                Param::Vector3::Zero(),                             // targetVec2 (unused)
                Param::Vector3::Zero()                              // slewRates
            },
            
            // [11] CROSS-TRACK POINTING: +Y to Nadir (side-looking)
            // Duration: 30 minutes
            Param::MissionEvent{
                1590.0,                                            // t_start
                1770.0,                                            // t_end
                Param::PointingMode::POINT,                         // mode
                Param::Vector3::UnitY(),                            // face: +Y axis (side-looking)
                Param::TargetType::NADIR,                           // target: Nadir
                Param::Vector3::Zero(),                             // targetVec (unused)
                Param::Vector3::UnitZ(),                            // secondFace: +Z axis
                Param::TargetType::SUN,                             // secondTarget: Sun
                Param::Vector3::Zero(),                             // targetVec2 (unused)
                Param::Vector3::Zero()                              // slewRates
            },
            
            // [12] FINAL SAFE MODE: Extended sun pointing for power
            // Duration: remainder of orbit 4 (~51 minutes)
            Param::MissionEvent{
                1770.0,                                            // t_start
                2000.0,                                            // t_end (end of orbit 4)
                Param::PointingMode::POINT,                         // mode
                Param::Vector3::UnitZ(),                            // face: +Z axis
                Param::TargetType::SUN,                             // target: Sun
                Param::Vector3::Zero(),                             // targetVec (unused)
                Param::Vector3::UnitX(),                            // secondFace: +X axis
                Param::TargetType::NADIR,                           // secondTarget: Nadir
                Param::Vector3::Zero(),                             // targetVec2 (unused)
                Param::Vector3::Zero()                              // slewRates
            }
        };
    }

    // Helper: Get total mission duration
    inline Param::Real getMissionDuration() {
        return 2000.0;  // 4 orbits
    }

    // Helper: Get number of events
    inline size_t getEventCount() {
        return 13;
    }

} // namespace Mission

#endif // CORE_SCHEDULE_HPP