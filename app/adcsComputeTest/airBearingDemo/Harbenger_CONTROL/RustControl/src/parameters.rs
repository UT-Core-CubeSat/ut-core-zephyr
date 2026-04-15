use nalgebra as na;
use std::f64::consts::PI;

pub struct Parameters {
    // Simulation parameters
    pub sample: f64,
    pub length: f64,
    pub title: String,
    pub plots: (usize, usize),
    pub time: f64,
    
    // Model matrices
    pub c_m: na::Matrix4x7<f64>,
    pub d_m: na::Matrix4x3<f64>,
    pub c_r: na::Matrix3x6<f64>,
    pub d_r: na::Matrix3x3<f64>,
    
    // Observer parameters
    pub p_0: na::Matrix6<f64>,
    pub g: na::Matrix6<f64>,
    pub q1: na::Matrix6<f64>,
    pub r1: na::Matrix3<f64>,
    
    // Controller parameters
    pub r: na::Matrix3<f64>,
    pub q: na::Matrix6<f64>,
    pub q_a: na::Matrix9<f64>,
    pub r_a: f64,
    pub n_a: na::Matrix9x3<f64>,
    
    // Inertia matrix
    pub inertia: na::Matrix3<f64>,
    
    // Sensor parameters
    pub sigma_gyro_x: f64,
    pub sigma_gyro_y: f64,
    pub sigma_gyro_z: f64,
    pub sigma_mag_x: f64,
    pub sigma_mag_y: f64,
    pub sigma_mag_z: f64,
    pub declination: f64,
    pub inclination: f64,
    pub field_strength: f64,
    pub sigma_star_quaternion: f64,
    pub ts_star_tracker: f64,
    
    // Dynamics matrices
    pub a: na::Matrix7<f64>,
    pub b: na::Matrix7x3<f64>,
    
    // Gain scheduling table
    pub gains: na::DMatrix<f64>,
}

impl Parameters {
    pub fn new() -> Self {
        // Basic parameters
        let sample = 0.01;
        let length = 150.0;
        let title = String::from("CubeSat");
        let plots = (3, 3);
        let time = 0.0;
        
        // Measurement matrices
        let c_m = na::Matrix4x7::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0
        );
        
        let d_m = na::Matrix4x3::zeros();
        
        let c_r = na::Matrix3x6::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0
        );
        
        let d_r = na::Matrix3x3::zeros();
        
        // Observer tuning parameters
        let p_0 = na::Matrix6::from_diagonal(&na::Vector6::new(0.1, 0.1, 0.1, 1.0, 1.0, 1.0));
        let g = na::Matrix6::identity();
        let q1 = na::Matrix6::from_diagonal(&na::Vector6::new(0.0, 0.0, 0.0, 1.0, 1.0, 1.0));
        let r1 = na::Matrix3::from_diagonal(&na::Vector3::new(0.1, 0.1, 0.1));
        
        // Controller tuning parameters
        let r = na::Matrix3::from_diagonal(&na::Vector3::new(1.0, 5.0, 5.0)) * 20.0;
        let q = na::Matrix6::from_diagonal(&na::Vector6::new(1.0, 10.0, 10.0, 0.0, 0.0, 0.0));
        let q_a = na::Matrix9::identity();
        let r_a = 5.0;
        let n_a = na::Matrix9x3::zeros();
        
        // Inertia matrix (scaled by 10^-6)
        let inertia = na::Matrix3::new(
            36857.5, 0.06, -7.6,
            0.06, 36771.47, 42.7,
            -7.6, 42.7, 7705.886
        ) * 1e-6;
        
        // Sensor parameters
        let sigma_gyro_x = 0.05;
        let sigma_gyro_y = 0.05;
        let sigma_gyro_z = 0.05;
        let sigma_mag_x = 0.02;
        let sigma_mag_y = 0.02;
        let sigma_mag_z = 0.02;
        let declination = 5.0 * PI / 180.0;  // Convert degrees to radians
        let inclination = 60.0 * PI / 180.0; // Convert degrees to radians
        let field_strength = 4.2e-5;
        let sigma_star_quaternion = 0.001;
        let ts_star_tracker = 1.0;
        
        // Initial state for linearization
        let q1 = 1.0;
        let q2 = 0.0;
        let q3 = 0.0;
        let q4 = 0.0;
        let wx = 0.0;
        let wy = 0.0;
        let wz = 0.0;
        
        // Extract inertia components
        let ixx = inertia[(0, 0)];
        let iyy = inertia[(1, 1)];
        let izz = inertia[(2, 2)];
        
        // Define A matrix
        let a = na::Matrix7::new(
            0.0, -wx/2.0, -wy/2.0, -wz/2.0, -q2/2.0, -q3/2.0, -q4/2.0,
            wx/2.0, 0.0, wz/2.0, -wy/2.0, q1/2.0, -q4/2.0, q3/2.0,
            wy/2.0, -wz/2.0, 0.0, wx/2.0, q4/2.0, q1/2.0, -q2/2.0,
            wz/2.0, wy/2.0, -wx/2.0, 0.0, -q3/2.0, q2/2.0, q1/2.0,
            0.0, 0.0, 0.0, 0.0, 0.0, (wz*(iyy - izz))/ixx, (wy*(iyy - izz))/ixx,
            0.0, 0.0, 0.0, 0.0, -(wz*(ixx - izz))/iyy, 0.0, -(wx*(ixx - izz))/iyy,
            0.0, 0.0, 0.0, 0.0, (wy*(ixx - iyy))/izz, (wx*(ixx - iyy))/izz, 0.0
        );
        
        // Define B matrix
        let b = na::Matrix7x3::new(
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            1.0/ixx, 0.0, 0.0,
            0.0, 1.0/iyy, 0.0,
            0.0, 0.0, 1.0/izz
        );
        
        // Load gains table (in the real implementation, you would load this from a file)
        // For now, creating a placeholder
        let gains = na::DMatrix::from_element(1, 31, 0.1);
        
        Parameters {
            sample,
            length,
            title,
            plots,
            time,
            c_m,
            d_m,
            c_r,
            d_r,
            p_0,
            g,
            q1,
            r1,
            r,
            q,
            q_a,
            r_a,
            n_a,
            inertia,
            sigma_gyro_x,
            sigma_gyro_y,
            sigma_gyro_z,
            sigma_mag_x,
            sigma_mag_y,
            sigma_mag_z,
            declination,
            inclination,
            field_strength,
            sigma_star_quaternion,
            ts_star_tracker,
            a,
            b,
            gains,
        }
    }
    
    /// Load gain table from file (would need to be implemented)
    pub fn load_gains(&mut self, filename: &str) {
        // In a real implementation, this would load the gains from a file
        // For now, just generate some placeholder data
        println!("Would load gains from file: {}", filename);
        
        // Create a simple 1x31 matrix of gain values
        self.gains = na::DMatrix::from_element(1, 31, 0.1);
    }
}