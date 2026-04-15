use nalgebra as na;
use crate::parameters::Parameters;
use crate::utils::quaternion;

pub struct Observer {
    // Observer states
    states_hat: na::Vector6<f64>,
    p: na::Matrix6<f64>,

    // Model matrices
    c_m: na::Matrix3x6<f64>,
    d_m: na::Matrix3<f64>,
    g: na::Matrix6<f64>,
    q: na::Matrix6<f64>,
    r: na::Matrix3<f64>,
    inertia: na::Matrix3<f64>,
    ts: f64,
}

impl Observer {
    pub fn new(parameters: &Parameters) -> Self {
        Observer {
            // Initialize observer states
            states_hat: na::Vector6::zeros(),
            p: parameters.p_0.clone(),

            // Initialize matrices
            c_m: na::Matrix3x6::new(
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0
            ),
            d_m: parameters.d_m.clone(),
            g: parameters.g.clone(),
            q: parameters.q1.clone(),
            r: parameters.r1.clone(),
            inertia: parameters.inertia.clone(),
            ts: parameters.sample,
        }
    }

    pub fn update(&mut self, inputs: &na::Vector3<f64>, measurements: &na::DVector<f64>) -> na::Vector7<f64> {
        // Propagate the EKF forward in time
        self.propagate_ekf(inputs, measurements);

        // Check if there is star tracker data
        if measurements.len() > 3 {
            // Extract star tracker measurements
            let star_measurements = na::Vector4::new(
                measurements[3], measurements[4], measurements[5], measurements[6]
            );
            
            self.star_update_ekf(inputs, &star_measurements);
        }

        // Convert current state estimate to quaternion form
        let mrp = na::Vector3::new(self.states_hat[0], self.states_hat[1], self.states_hat[2]);
        let quat = quaternion::mrp2quat(&mrp);
        
        // Combine quaternion and angular velocity into full state vector
        let mut x = na::Vector7::zeros();
        x[0] = quat[0];
        x[1] = quat[1];
        x[2] = quat[2];
        x[3] = quat[3];
        x[4] = self.states_hat[3];
        x[5] = self.states_hat[4];
        x[6] = self.states_hat[5];
        
        x
    }

    fn ekf_dif_eq(
        &self,
        x: &na::Vector6<f64>,
        p: &na::Matrix6<f64>,
        inputs: &na::Vector3<f64>,
        measurements: &na::DVector<f64>,
    ) -> (na::Vector6<f64>, na::Matrix6<f64>) {
        // Convert MRP to quaternion for linearization
        let mrp = na::Vector3::new(x[0], x[1], x[2]);
        let quat = quaternion::mrp2quat(&mrp);
        
        // Create full state vector for linearization
        let mut x_full = na::Vector7::zeros();
        x_full[0] = quat[0];
        x_full[1] = quat[1];
        x_full[2] = quat[2];
        x_full[3] = quat[3];
        x_full[4] = x[3];
        x_full[5] = x[4];
        x_full[6] = x[5];
        
        // Get linearized dynamics matrix
        let (a, _, _, _) = quaternion::modrod_state_space_fast(&x_full, inputs, &self.inertia);
        
        // Nonlinear differential equations for more accurate propagation
        let omega = na::Vector3::new(x[3], x[4], x[5]);
        let sigma = na::Vector3::new(x[0], x[1], x[2]);
        
        // Calculate MRP derivative
        let sigma_sq_norm = sigma.norm_squared();
        let identity = na::Matrix3::<f64>::identity();
        let cross_sigma = quaternion::cross_mat(&sigma);
        let sigma_sigma_t = sigma * sigma.transpose();
        
        let sigma_dot = 0.25 * ((1.0 - sigma_sq_norm) * identity + 
                               2.0 * cross_sigma + 
                               2.0 * sigma_sigma_t) * omega;
                               // Calculate angular acceleration
        let omega_dot = self.inertia.try_inverse().unwrap() * 
        (inputs - omega.cross(&(self.inertia * omega)));
    
    // Combine into state derivative
    let f = na::Vector6::new(
        sigma_dot[0], sigma_dot[1], sigma_dot[2],
        omega_dot[0], omega_dot[1], omega_dot[2]
    );
    
    // Extract gyroscope measurements
    let y_m = na::Vector3::new(measurements[0], measurements[1], measurements[2]);
    
    // Kalman gain
    let k_k = p * self.c_m.transpose() * self.r.try_inverse().unwrap();
    
    // Predicted output
    let y_hat = self.c_m * x;
    
    // State derivative
    let x_hat_dot = f + k_k * (y_m - y_hat);
    
    // Covariance derivative
    let p_dot = a * p + p * a.transpose() - 
                p * self.c_m.transpose() * self.r.try_inverse().unwrap() * self.c_m * p + 
                self.g * self.q * self.g.transpose();
    
    (x_hat_dot, p_dot)
}

fn star_update_ekf(&mut self, _inputs: &na::Vector3<f64>, measurements: &na::Vector4<f64>) {
    // Convert quaternion to MRP
    let mrp_measured = quaternion::quat2mrp(measurements);
    
    // Define observation matrix for star tracker (measures attitude directly)
    let c = na::Matrix3x6::new(
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0
    );
    
    // Extract current attitude estimate
    let mrp_estimated = na::Vector3::new(
        self.states_hat[0], self.states_hat[1], self.states_hat[2]
    );
    
    // Update covariance
    let innovation_cov = c * self.p * c.transpose() + self.r;
    self.p = self.p - self.p * c.transpose() * innovation_cov.try_inverse().unwrap() * c * self.p;
    
    // Kalman gain
    let k_l = self.p * c.transpose() * self.r.try_inverse().unwrap();
    
    // Innovation (measurement error)
    let e_y = mrp_measured - mrp_estimated;
    
    // Update state estimate
    self.states_hat = self.states_hat + k_l * e_y;
}

fn propagate_ekf(&mut self, inputs: &na::Vector3<f64>, measurements: &na::DVector<f64>) {
    // RK4 integration for the EKF
    // Step 1
    let (x_hat_dot_1, p_dot_1) = self.ekf_dif_eq(
        &self.states_hat, &self.p, inputs, measurements
    );
    
    // Step 2
    let x_hat_2 = self.states_hat + x_hat_dot_1 * (self.ts / 2.0);
    let p_2 = self.p + p_dot_1 * (self.ts / 2.0);
    let (x_hat_dot_2, p_dot_2) = self.ekf_dif_eq(
        &x_hat_2, &p_2, inputs, measurements
    );
    
    // Step 3
    let x_hat_3 = self.states_hat + x_hat_dot_2 * (self.ts / 2.0);
    let p_3 = self.p + p_dot_2 * (self.ts / 2.0);
    let (x_hat_dot_3, p_dot_3) = self.ekf_dif_eq(
        &x_hat_3, &p_3, inputs, measurements
    );
    
    // Step 4
    let x_hat_4 = self.states_hat + x_hat_dot_3 * self.ts;
    let p_4 = self.p + p_dot_3 * self.ts;
    let (x_hat_dot_4, p_dot_4) = self.ekf_dif_eq(
        &x_hat_4, &p_4, inputs, measurements
    );
    
    // Combine steps
    self.states_hat = self.states_hat + 
        (x_hat_dot_1 + x_hat_dot_2 * 2.0 + x_hat_dot_3 * 2.0 + x_hat_dot_4) * (self.ts / 6.0);
    
    self.p = self.p + 
        (p_dot_1 + p_dot_2 * 2.0 + p_dot_3 * 2.0 + p_dot_4) * (self.ts / 6.0);
}
}