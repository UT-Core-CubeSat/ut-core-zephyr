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
                    // True States (17)
                    << "rx,ry,rz,vx,vy,vz,q0,q1,q2,q3,wx,wy,wz,rw1,rw2,rw3,rw4,"
                    // Estimates (17)
                    << "rx_est,ry_est,rz_est,vx_est,vy_est,vz_est,"
                    << "q0_est,q1_est,q2_est,q3_est,wx_est,wy_est,wz_est,"
                    << "rw1_est,rw2_est,rw3_est,rw4_est,"
                    // Reference (10)
                    << "q0_ref,q1_ref,q2_ref,q3_ref,wx_ref,wy_ref,wz_ref,ax_ref,ay_ref,az_ref,"
                    // Inputs (7)
                    << "tau_w1,tau_w2,tau_w3,tau_w4,m_x,m_y,m_z,"
                    // Measurements (29)
                    << "meas_ax,meas_ay,meas_az,"
                    << "meas_gx,meas_gy,meas_gz,"
                    << "meas_st_q0,meas_st_q1,meas_st_q2,meas_st_q3,"
                    << "meas_css1,meas_css2,meas_css3,meas_css4,meas_css5,meas_css6,"
                    << "meas_mx,meas_my,meas_mz,"
                    << "meas_gps_r1,meas_gps_r2,meas_gps_r3,meas_gps_v1,meas_gps_v2,meas_gps_v3,"
                    << "meas_rw1,meas_rw2,meas_rw3,meas_rw4,"
                    // Model States (7)
                    << "q_m0,q_m1,q_m2,q_m3,omega_m_x,omega_m_y,omega_m_z,"
                    // Mode
                    << "mode\n";
            }
        }
    }

    void log(Real t,
             const Param::Vector17& true_state,
             const Param::Vector17& est_state,
             const Param::Vector10& ref,
             const Param::Vector7& input,
             const Param::Vector29& meas,
             const Param::Vector7& model,
             int mode)
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(9) << t << ",";

        auto write_vec = [&](const auto& v, int size) {
            for (int i = 0; i < size; ++i) ss << v(i) << ",";
        };

        write_vec(true_state, 17);
        write_vec(est_state, 17);
        write_vec(ref, 10);
        write_vec(input, 7);
        write_vec(meas, 29);
        write_vec(model, 7);
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
// HELPER: Convert plant measurements (Vector29) to ADCS::SensorData
// ============================================================================
ADCS::SensorData packSensorData(const Param::Vector29& meas, PlantParam::TimeReal unix_time) {
    ADCS::SensorData data;
    data.unix_time = unix_time;
    
    // Gyro: indices 3-5
    data.gyro(0) = meas(3);
    data.gyro(1) = meas(4);
    data.gyro(2) = meas(5);
    
    // Star tracker quaternion: indices 6-9
    data.star_quat(0) = meas(6);
    data.star_quat(1) = meas(7);
    data.star_quat(2) = meas(8);
    data.star_quat(3) = meas(9);
    
    // CSS currents: indices 10-15
    data.css_currents(0) = meas(10);
    data.css_currents(1) = meas(11);
    data.css_currents(2) = meas(12);
    data.css_currents(3) = meas(13);
    data.css_currents(4) = meas(14);
    data.css_currents(5) = meas(15);
    
    // Magnetometer: indices 16-18
    data.magnetometer(0) = meas(16);
    data.magnetometer(1) = meas(17);
    data.magnetometer(2) = meas(18);
    
    // GPS ECEF (pos + vel): indices 19-24
    data.gps_ecef(0) = meas(19);
    data.gps_ecef(1) = meas(20);
    data.gps_ecef(2) = meas(21);
    data.gps_ecef(3) = meas(22);
    data.gps_ecef(4) = meas(23);
    data.gps_ecef(5) = meas(24);
    
    // Wheel speeds: indices 25-28
    data.wheel_speeds(0) = meas(25);
    data.wheel_speeds(1) = meas(26);
    data.wheel_speeds(2) = meas(27);
    data.wheel_speeds(3) = meas(28);
    
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

    SimLogger logger({"simulation_data.csv", "../../MATLAB/simulation_data.csv"});

    // State tracking
    Param::Vector17 states_true = dynamics.getStates();
    Param::Vector17 states_hat = states_true;
    Param::Vector10 reference = Param::Vector10::Zero();
    Param::Vector7 tau_all = Param::Vector7::Zero();
    Param::Vector7 states_m = Param::Vector7::Zero();
    Param::Vector29 measurements = Param::Vector29::Zero();

    std::cout << "Running Simulation..." << std::endl;

    // ========================================================================
    // MISSION SCHEDULE (Plant owns this now!)
    // ========================================================================
    auto getCommand = [](Real t) -> ADCS::Command {
        ADCS::Command cmd;
        
        if (t < static_cast<Real>(60.0)) {
            // Phase 1: Detumble
            cmd.mode = ADCS::MissionMode::SAFE;
        } 
        else if (t < static_cast<Real>(500.0)) {
            // Phase 2: Sun pointing for power
            cmd.mode = ADCS::MissionMode::STANDBY;
        }
        else if (t < static_cast<Real>(800.0)) {
            // Phase 3: Downlink to ground station
            cmd.mode = ADCS::MissionMode::DOWNLINK;
            cmd.target.latitude = static_cast<Real>(32.9);    // San Diego
            cmd.target.longitude = static_cast<Real>(-117.2);
            cmd.target.altitude = static_cast<Real>(0.0);
        }
        else if (t < static_cast<Real>(1200.0)) {
            // Phase 4: Nadir imaging
            cmd.mode = ADCS::MissionMode::IMAGING;
            cmd.track_target = false;  // Nadir pointing
        }
        else {
            // Phase 5: Back to standby
            cmd.mode = ADCS::MissionMode::STANDBY;
        }
        
        return cmd;
    };

    // Initial log
    logger.log(t, states_true, states_hat, reference, tau_all, measurements, 
               states_m, static_cast<int>(ADCS::MissionMode::SAFE));

    while (t < t_end) {
        while (t <= t_next_plot) {
            Param::Vector17 states_dot = dynamics.getStatesDot();
            measurements = sensors.measurements(states_dot, dynamics.getStates(), t);

            TimeReal unix_time = epoch + t;
            ADCS::SensorData sensorData = packSensorData(measurements, unix_time);
            
            // Get command from schedule
            ADCS::Command cmd = getCommand(t);

            // Call Core
            ADCS::AdcsOutput actuators = adcsCore.update(sensorData, cmd);

            // Remove these later !!!
            states_hat = actuators.states_hat;
            reference = actuators.reference;
            states_m = actuators.states_m;

            // Extract for dynamics
            tau_all(0) = actuators.wheel_torque(0);
            tau_all(1) = actuators.wheel_torque(1);
            tau_all(2) = actuators.wheel_torque(2);
            tau_all(3) = actuators.wheel_torque(3);
            tau_all(4) = actuators.mtq_dipole(0);
            tau_all(5) = actuators.mtq_dipole(1);
            tau_all(6) = actuators.mtq_dipole(2);

            // Map MissionMode to PointingMode for dynamics
            Param::PointingMode dyn_mode = Param::PointingMode::OFF;
            if (cmd.mode == ADCS::MissionMode::SAFE) {
                dyn_mode = Param::PointingMode::DETUMBLE;
            } else if (cmd.mode != ADCS::MissionMode::SAFE) {
                dyn_mode = Param::PointingMode::POINT;
            }

            dynamics.update(tau_all);
            t += dt;
        }

        states_true = dynamics.getStates();
        ADCS::Command current_cmd = getCommand(t);
        logger.log(t, states_true, states_hat, reference, tau_all, measurements, 
                   states_m, static_cast<int>(current_cmd.mode));
        t_next_plot = t + t_plot;
    }

    logger.close();
    std::cout << "Simulation Complete." << std::endl;
    return 0;
}