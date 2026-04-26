// ============================================================================
// PLANT MAIN - ADCS Simulation Harness
// ============================================================================
// This file simulates the satellite plant (dynamics, sensors, environment)
// and interfaces with ADCSCore as a black box. The Core receives sensor
// measurements and returns actuator commands, just like flight software.
// ============================================================================

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <cstdlib>

// Plant includes
#include "Plant_Parameters.hpp"
#include "Dynamics.hpp"
#include "Sensors.hpp"

// Core includes (black box)
#include "../components/ADCSCore.hpp"

// ============================================================================
// CSV DATA LOGGER
// ============================================================================
struct SimLogger {
    std::vector<std::ofstream> files;

    SimLogger(const std::vector<std::string>& filenames) {
        files.reserve(filenames.size());
        for (const auto& name : filenames) {
            files.emplace_back(name);
            if (files.back().is_open()) {
                files.back()
                    << "t,"
                    // True States (11): q(4), omega(3), omega_wheel(4)
                    << "q0,q1,q2,q3,wx,wy,wz,rw1,rw2,rw3,rw4,"
                    // Estimates (11)
                    << "q0_est,q1_est,q2_est,q3_est,wx_est,wy_est,wz_est,"
                    << "rw1_est,rw2_est,rw3_est,rw4_est,"
                    // Reference (10)
                    << "q0_ref,q1_ref,q2_ref,q3_ref,wx_ref,wy_ref,wz_ref,ax_ref,ay_ref,az_ref,"
                    // Inputs (7)
                    << "tau_w1,tau_w2,tau_w3,tau_w4,m_x,m_y,m_z,"
                    // Measurements (13)
                    << "meas_ax,meas_ay,meas_az,"
                    << "meas_wx,meas_wy,meas_wz,"
                    << "meas_mx,meas_my,meas_mz,"
                    << "meas_rw1,meas_rw2,meas_rw3,meas_rw4,"
                    // Model States (7)
                    << "q_m0,q_m1,q_m2,q_m3,omega_m_x,omega_m_y,omega_m_z,"
                    // Disturbance Torques (6) - NEW
                    << "tau_grav_x,tau_grav_y,tau_grav_z,"
                    << "tau_dist_x,tau_dist_y,tau_dist_z,"
                    // Per-Face MTQ Data (8): currents (A) and reference fields (T)
                    << "I_Xpos,I_Xneg,I_Ypos,I_Yneg,"
                    << "B_Xpos_ref,B_Xneg_ref,B_Ypos_ref,B_Yneg_ref,"
                    // Innovation Diagnostics (8): accel and mag residuals
                    << "accel_innov_x,accel_innov_y,accel_innov_z,accel_innov_norm,"
                    << "mag_innov_x,mag_innov_y,mag_innov_z,mag_innov_norm,"
                    // Mode
                    << "mode\n";
            }
        }
    }

    void log(Real t,
             const Param::Vector11& true_state,
             const Param::Vector11& est_state,
             const Param::Vector10& ref,
             const Param::Vector7& input,
             const Param::Vector13& meas,
             const Param::Vector7& model,
             const Param::Vector3& tau_grav,   // NEW
             const Param::Vector3& tau_dist,   // NEW
            const Param::Vector4& mtq_face_current,
            const Param::Vector4& mtq_face_b_ref,
            const Param::Vector3& accel_innov,
            Real accel_innov_norm,
            const Param::Vector3& mag_innov,
            Real mag_innov_norm,
            int mode)
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(9) << t << ",";

        auto write_vec = [&](const auto& v, int size) {
            for (int i = 0; i < size; ++i) ss << v(i) << ",";
        };

        write_vec(true_state, 11);
        write_vec(est_state, 11);
        write_vec(ref, 10);
        write_vec(input, 7);
        write_vec(meas, 13);
        write_vec(model, 7);
        write_vec(tau_grav, 3);   // NEW
        write_vec(tau_dist, 3);   // NEW
        write_vec(mtq_face_current, 4);
        write_vec(mtq_face_b_ref, 4);
        // Innovation diagnostics
        write_vec(accel_innov, 3);
        ss << accel_innov_norm << ",";
        write_vec(mag_innov, 3);
        ss << mag_innov_norm << ",";
        ss << mode << "\n";

        std::string line = ss.str();
        for (auto& f : files) {
            if (f.is_open()) f << line;
        }
    }

    void close() {
        for (auto& f : files)
            if (f.is_open()) f.close();
    }
};

// ============================================================================
// HELPER: Convert plant measurements (Vector13) to ADCS::SensorData
// ============================================================================
ADCS::SensorData packSensorData(const Param::Vector13& meas, PlantParam::TimeReal unix_time) {
    ADCS::SensorData data;
    data.unix_time = unix_time;
    
    // Accelerometer: indices 0-2
    data.accelerometer(0) = meas(0);
    data.accelerometer(1) = meas(1);
    data.accelerometer(2) = meas(2);

    // Gyro: indices 3-5
    data.gyro(0) = meas(3);
    data.gyro(1) = meas(4);
    data.gyro(2) = meas(5);
    
    // Magnetometer: indices 6-8
    data.magnetometer(0) = meas(6);
    data.magnetometer(1) = meas(7);
    data.magnetometer(2) = meas(8);
    
    // Wheel speeds: indices 9-12
    data.wheel_speeds(0) = meas(9);
    data.wheel_speeds(1) = meas(10);
    data.wheel_speeds(2) = meas(11);
    data.wheel_speeds(3) = meas(12);
    
    return data;
}

// ============================================================================
// MAIN SIMULATION LOOP
// ============================================================================
// ... (SimLogger stays the same)

int main() {
    using namespace PlantParam;
    std::cout << "=== ADCS Plant Simulation ===" << std::endl;

    std::srand(42);

    Dynamics dynamics;
    SensorsClass sensors;
    ADCS::Core adcsCore;

    Real t = SimTime::t_start;
    Real t_end = SimTime::t_end;
    Real dt = SimTime::Ts;
    Real t_plot = SimTime::t_plot;
    Real t_next_plot = static_cast<Real>(0.0);
    TimeReal epoch = SimTime::epoch_timestamp;

    SimLogger logger({"simulation_data.csv", "../../../simulation/simulation_data.csv"});

    // State tracking
    Param::Vector11 states_true = dynamics.getStates();
    Param::Vector11 states_hat = states_true;
    Param::Vector10 reference = Param::Vector10::Zero();
    Param::Vector7 tau_all = Param::Vector7::Zero();
    Param::Vector7 states_m = Param::Vector7::Zero();
    Param::Vector13 measurements = Param::Vector13::Zero();
    Param::Vector3 tau_grav = Param::Vector3::Zero();    // NEW
    Param::Vector3 tau_dist = Param::Vector3::Zero();    // NEW
    Param::Vector4 mtq_face_current = Param::Vector4::Zero();
    Param::Vector4 mtq_face_b_ref = Param::Vector4::Zero();
    Param::Vector3 accel_innov = Param::Vector3::Zero();
    Real accel_innov_norm = static_cast<Real>(0.0);
    Param::Vector3 mag_innov = Param::Vector3::Zero();
    Real mag_innov_norm = static_cast<Real>(0.0);
    ADCS::MissionMode active_mode = ADCS::MissionMode::OFF;

    std::cout << "Running Simulation..." << std::endl;

    auto getCommand = [](Real t) {
        ADCS::Command cmd;
        // Demo sequence: OFF for startup, DETUMBLE for B-dot, then BOTH for NDI + MTQ desat.
        if (t < static_cast<Real>(0.5)) {
            cmd.mode = ADCS::MissionMode::OFF;
        } else if (t < static_cast<Real>(60.0)) {
            cmd.mode = ADCS::MissionMode::BOTH;
        } else {
            cmd.mode = ADCS::MissionMode::BOTH;
        }
        return cmd;
    };

    // Initial log
    logger.log(t, states_true, states_hat, reference, tau_all, measurements, 
               states_m, tau_grav, tau_dist, mtq_face_current, mtq_face_b_ref,
               accel_innov, accel_innov_norm, mag_innov, mag_innov_norm,
               static_cast<int>(ADCS::MissionMode::OFF));
    while (t < t_end) {
        while (t <= t_next_plot) {
            Param::Vector11 states_dot = dynamics.getStatesDot();
            measurements = sensors.measurements(states_dot, dynamics.getStates(), t);

            TimeReal unix_time = epoch + t;
            ADCS::SensorData sensorData = packSensorData(measurements, unix_time);
            ADCS::Command cmd = getCommand(t);

            // Call Core
            ADCS::AdcsOutput actuators = adcsCore.update(sensorData, cmd);
            active_mode = actuators.current_mode;

            // Reconstruct states_hat for logging from available estimate fields
            states_hat.setSegment(0, actuators.attitude_est);
            states_hat.setSegment(4, actuators.rate_est);
            states_hat.setSegment(7, measurements.segment<4>(9)); // wheel speeds from meas

            // Mirror ADCSCore reference construction so CSV reflects component behavior.
            reference = Param::Vector10::Zero();
            reference(0) = static_cast<Real>(1.0);
            reference(3) = states_hat(3);

            // Convert ADCSCore wheel RPM commands → torque commands for the plant dynamics.
            // Implements a deadbeat proportional RPM controller clamped to actuator limits,
            // matching what the embedded motor controller does with the RPM setpoint.
            {
                Param::Vector4 omega_w_true = dynamics.getStates().segment<4>(7);
                Param::Vector4 omega_w_cmd = actuators.wheel_rpm *
                    (static_cast<Real>(2.0) * PlantParam::PI / static_cast<Real>(60.0));
                Param::Real kp = PlantParam::Actuators::I_wheel / dt;
                Param::Vector4 tau_cmd = kp * (omega_w_cmd - omega_w_true);
                for (int i = 0; i < 4; ++i) {
                    tau_cmd(i) = std::max(-PlantParam::Actuators::tau_w_max,
                                 std::min( PlantParam::Actuators::tau_w_max, tau_cmd(i)));
                }
                tau_all(0) = tau_cmd(0);
                tau_all(1) = tau_cmd(1);
                tau_all(2) = tau_cmd(2);
                tau_all(3) = tau_cmd(3);
            }
            tau_all(4) = actuators.mtq_dipole(0);
            tau_all(5) = actuators.mtq_dipole(1);
            tau_all(6) = actuators.mtq_dipole(2);

            // mtq_face_current, mtq_face_b_ref, and innovation diagnostics are not
            // provided by this ADCSCore build — leave them as zero for the logger.

            dynamics.update(tau_all);
            t += dt;
        }

        states_true = dynamics.getStates();
        // states_hat = states_true; // For logging equivalence only
        tau_grav = dynamics.getTauGravity();      // NEW
        tau_dist = dynamics.getTauDisturbance();  // NEW
        logger.log(t, states_true, states_hat, reference, tau_all, measurements, 
                   states_m, tau_grav, tau_dist, mtq_face_current, mtq_face_b_ref,
                   accel_innov, accel_innov_norm, mag_innov, mag_innov_norm,
                   static_cast<int>(active_mode));
        t_next_plot = t + t_plot;
    }

    logger.close();
    std::cout << "Simulation Complete." << std::endl;
    return 0;
}