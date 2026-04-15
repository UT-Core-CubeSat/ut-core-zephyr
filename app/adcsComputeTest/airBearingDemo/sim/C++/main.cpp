#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <chrono>
#include <iomanip>

#include "Parameters.hpp"
#include "Dynamics.hpp"
#include "Sensors.hpp"
#include "Observer.hpp"
#include "ControllerManager.hpp"
#include "ReferenceGenerator.hpp"

// =========================================================================
// CSV DATA LOGGER
// =========================================================================
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
                     << "rx_est,ry_est,rz_est,vx_est,vy_est,vz_est,q0_est,q1_est,q2_est,q3_est,wx_est,wy_est,wz_est,rw1_est,rw2_est,rw3_est,rw4_est,"
                     // Reference (10)
                     << "q0_ref,q1_ref,q2_ref,q3_ref,wx_ref,wy_ref,wz_ref,ax_ref,ay_ref,az_ref,"
                     // Inputs (7)
                     << "tau_w1,tau_w2,tau_w3,tau_w4,m_x,m_y,m_z,"
                     // Measurements (29)
                     << "meas_ax,meas_ay,meas_az,"             // 0-2 (Accel)
                     << "meas_gx,meas_gy,meas_gz,"             // 3-5 (Gyro)
                     << "meas_st_q0,meas_st_q1,meas_st_q2,meas_st_q3," // 6-9 (Star Tracker)
                     << "meas_css1,meas_css2,meas_css3,meas_css4,meas_css5,meas_css6," // 10-15 (Sun)
                     << "meas_mx,meas_my,meas_mz,"             // 16-18 (Mag)
                     << "meas_gps_r1,meas_gps_r2,meas_gps_r3,meas_gps_v1,meas_gps_v2,meas_gps_v3," // 19-24 (GPS)
                     << "meas_rw1,meas_rw2,meas_rw3,meas_rw4," // 25-28 (Wheels)
                     // Model States (7) - Reference model: q_m(0:3), omega_m(0:2)
                     << "q_m0,q_m1,q_m2,q_m3,omega_m_x,omega_m_y,omega_m_z,"
                     // Mode
                     << "mode\n"; 
            }
        }
    }

    void log(double t, 
             const Param::Vector17& true_state, 
             const Param::Vector17& est_state,
             const Param::Vector10& ref,
             const Param::Vector7& input,
             const Param::Vector29& meas,  // <--- NEW
             const Param::Vector7& model,  // <--- NEW
             int mode) 
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(9) << t << ",";
        
        auto write_vec = [&](const auto& v, int size) {
            for(int i=0; i<size; ++i) ss << v(i) << ",";
        };

        write_vec(true_state, 17);
        write_vec(est_state, 17);
        write_vec(ref, 10);
        write_vec(input, 7);
        write_vec(meas, 29);  // <--- NEW
        write_vec(model, 7);  // <--- NEW
        ss << mode << "\n";

        std::string line = ss.str();
        for (auto& f : files) {
            if (f.is_open()) f << line;
        }
    }

    void close() {
        for (auto& f : files) if (f.is_open()) f.close();
    }
};

// =========================================================================
// MAIN LOOP
// =========================================================================
int main() {
    using namespace Param;
    std::cout << "Initializing Simulation..." << std::endl;

    // Initialize RNG with seed 42 BEFORE any sensor calls (matches MATLAB's rng(42))
    // This ensures sensor noise sequences are identical between C++ and MATLAB
    std::srand(42);
    
    Dynamics dynamics;
    SensorsClass sensors;
    ObserverClass observer;
    ControllerManager controller;
    ReferenceGenerator refGen;

    Real t = SimTime::t_start;
    Real t_end = SimTime::t_end;
    Real dt = SimTime::Ts;
    Real t_plot = SimTime::t_plot;
    Real t_next_plot = 0.0;

    // Output to both local C++ folder and sibling MATLAB folder
    SimLogger logger({ "simulation_data.csv", "../MATLAB/simulation_data.csv" });

    // Persistent Variables
    Vector17 states_true = dynamics.getStates(); 
    Vector17 states_hat = states_true;
    Vector10 reference = Vector10::Zero();
    Vector7 tau_all = Vector7::Zero();
    Vector7 states_m = Vector7::Zero();        // <--- NEW
    Vector29 measurements = Vector29::Zero();  // <--- NEW
    PointingMode mode = PointingMode::OFF;

    std::cout << "Running Simulation (0 to " << t_end << "s)..." << std::endl;

    // Initial Log (t=0, before any dynamics updates)
    logger.log(t, states_true, states_hat, reference, tau_all, measurements, states_m, (int)mode);

    while (t < t_end) {

        while (t <= t_next_plot) {
            
            Vector17 states_dot = dynamics.getStatesDot();

            auto ref_out = refGen.update(dynamics.getStates(), t);
            reference = ref_out.reference;
            mode = ref_out.mode;

            // 1. Update Sensors
            measurements = sensors.measurements(states_dot, dynamics.getStates(), t);

            // 2. Update Observer
            states_hat = observer.update(measurements, t); 
            //states_hat = states_true; 

            // 3. Update Controller
            auto ctrl_out = controller.update(states_hat, reference, measurements, mode);
            tau_all = ctrl_out.tau;
            states_m = ctrl_out.states_m; // <--- Capture Model States

            // 4. Update Dynamics (this changes states)
            dynamics.update(ctrl_out.tau, mode);
            t += dt;
        }

        // Log AFTER dynamics update (matching MATLAB timing)
        states_true = dynamics.getStates();
        logger.log(t, states_true, states_hat, reference, tau_all, measurements, states_m, (int)mode);
        t_next_plot = t + t_plot;
    }

    logger.close();
    std::cout << "Simulation Complete." << std::endl;
    return 0;
}