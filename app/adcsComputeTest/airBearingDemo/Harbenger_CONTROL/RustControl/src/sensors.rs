use nalgebra as na;
use rand::distributions::{Distribution, Normal};
use crate::parameters::Parameters;
use crate::utils::quaternion;

pub struct Sensors {
    // IMU noise parameters
    sigma_gyro_x: f64,
    sigma_gyro_y: f64,
    sigma_gyro_z: f64,
    
    // Magnetometer noise parameters
    sigma_mag_x: f64,
    sigma_mag_y: f64,
    sigma_mag_z: f64,
    
    // Magnetic field parameters
    declination: f64,
    inclination: f64,
    field_strength: f64,
    
    // Star tracker noise parameter
    sigma_star_quaternion: f64,
    
    // Timing parameters
    ts_star_tracker: f64,
    last_star_tracker_update_t: f64,
    
    // Last star tracker measurement (for plotting)
    last_star_tracker_update: na::Vector4<f64>,
    
    // Random number generators
    rng_gyro_x: Normal<f64>,
    rng_gyro_y: Normal<f64>,
    rng_gyro_z: Normal<f64>,
    rng_star: Normal<f64>,
}

impl Sensors {
    pub fn new(parameters: &Parameters) -> Self {
        Sensors {
            // IMU noise parameters
            sigma_gyro_x: parameters.sigma_gyro_x,
            sigma_gyro_y: parameters.sigma_gyro_y,
            sigma_gyro_z: parameters.sigma_gyro_z,
            
            // Magnetometer noise parameters
            sigma_mag_x: parameters.sigma_mag_x,
            sigma_mag_y: parameters.sigma_mag_y,
            sigma_mag_z: parameters.sigma_mag_z,
            
            // Magnetic field parameters
            declination: parameters.declination,
            inclination: parameters.inclination,
            field_strength: parameters.field_strength,
            
            // Star tracker noise
            sigma_star_quaternion: parameters.sigma_star_quaternion,
            
            // Timing
            ts_star_tracker: parameters.ts_star_tracker,
            last_star_tracker_update_t: f64::NEG_INFINITY,
            
            // Last measurement
            last_star_tracker_update: na::Vector4::new(1.0, 0.0, 0.0, 0.0),
            
            // Initialize random number generators
            rng_gyro_x: Normal::new(0.0, parameters.sigma_gyro_x),
            rng_gyro_y: Normal::new(0.0, parameters.sigma_gyro_y),
            rng_gyro_z: Normal::new(0.0, parameters.sigma_gyro_z),
            rng_star: Normal::new(0.0, parameters.sigma_star_quaternion),
        }
    }
    
    pub fn update(
        &mut self, 
        states: &na::Vector7<f64>, 
        _magnetic_field: f64, 
        t: f64,
        force_star_tracker: bool
    ) -> na::DVector<f64> {
        // Get gyroscope measurements
        let (gyro_x, gyro_y, gyro_z) = self.gyroscope(states);
        
        // Initialize measurement vector with gyroscope readings
        let mut y = na::DVector::from_vec(vec![gyro_x, gyro_y, gyro_z]);
        
        // Add star tracker measurements if it's time or forced
        if (t - self.last_star_tracker_update_t) >= self.ts_star_tracker || force_star_tracker {
            let (quat_w, quat_x, quat_y, quat_z) = self.star_tracker(states);
            
            // Add quaternion measurements to the vector
            y = na::DVector::from_vec(vec![
                gyro_x, gyro_y, gyro_z, 
                quat_w, quat_x, quat_y, quat_z
            ]);
            
            // Update last measurement time if not forced
            if !force_star_tracker {
                self.last_star_tracker_update_t = t;
                self.last_star_tracker_update = na::Vector4::new(quat_w, quat_x, quat_y, quat_z);
            }
        }
        
        y
    }
    
    fn gyroscope(&mut self, states: &na::Vector7<f64>) -> (f64, f64, f64) {
        // Extract angular velocity
        let wx = states[4];
        let wy = states[5];
        let wz = states[6];
        
        // Generate random noise
        let eta_gyro_x = self.rng_gyro_x.sample(&mut rand::thread_rng());
        let eta_gyro_y = self.rng_gyro_y.sample(&mut rand::thread_rng());
        let eta_gyro_z = self.rng_gyro_z.sample(&mut rand::thread_rng());
        
        // Add noise to measurements
        let y_gyro_x = wx + eta_gyro_x;
        let y_gyro_y = wy + eta_gyro_y;
        let y_gyro_z = wz + eta_gyro_z;
        
        (y_gyro_x, y_gyro_y, y_gyro_z)
    }
    
    fn star_tracker(&mut self, states: &na::Vector7<f64>) -> (f64, f64, f64, f64) {
        // Extract quaternion
        let qw = states[0];
        let qx = states[1];
        let qy = states[2];
        let qz = states[3];
        
        // Create a small random rotation quaternion to model star tracker noise
        let eta_angle = self.rng_star.sample(&mut rand::thread_rng());
        
        // Generate random axis
        let mut rng = rand::thread_rng();
        let eta_axis_x = self.rng_star.sample(&mut rng);
        let eta_axis_y = self.rng_star.sample(&mut rng);
        let eta_axis_z = self.rng_star.sample(&mut rng);
        
        // Normalize the axis
        let axis_norm = (eta_axis_x * eta_axis_x + 
                         eta_axis_y * eta_axis_y + 
                         eta_axis_z * eta_axis_z).sqrt();
        
        let eta_axis = na::Vector3::new(
            eta_axis_x / axis_norm,
            eta_axis_y / axis_norm,
            eta_axis_z / axis_norm
        );
        
        // Create noise quaternion
        let eta_quat = na::Vector4::new(
            (eta_angle / 2.0).cos(),
            (eta_angle / 2.0).sin() * eta_axis[0],
            (eta_angle / 2.0).sin() * eta_axis[1],
            (eta_angle / 2.0).sin() * eta_axis[2]
        );
        
        // Apply noise via quaternion multiplication
        let true_quat = na::Vector4::new(qw, qx, qy, qz);
        let noisy_quat = quaternion::quatmultiply(&true_quat, &eta_quat);
        
        // Normalize
        let quat_norm = noisy_quat.norm();
        let normalized_quat = noisy_quat / quat_norm;
        
        (normalized_quat[0], normalized_quat[1], normalized_quat[2], normalized_quat[3])
    }
}