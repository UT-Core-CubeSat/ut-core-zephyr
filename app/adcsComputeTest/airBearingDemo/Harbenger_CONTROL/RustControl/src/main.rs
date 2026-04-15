mod animation;
mod controller;
mod dynamics;
mod integrator;
mod observer;
mod parameters;
mod plotter;
mod sensors;
mod utils;

use nalgebra as na;
use std::collections::HashMap;
use std::time::Duration;
use std::thread;

use animation::Animation;
use controller::Controller;
use dynamics::Dynamics;
use observer::Observer;
use parameters::Parameters;
use plotter::{Plotter, PlotVariable, rgb};
use sensors::Sensors;
use utils::quaternion;

fn main() {
    // Initialize parameters
    let parameters = Parameters::new();
    
    // Create components
    let mut animation = Animation::new();
    let mut observer = Observer::new(&parameters);
    let controller = Controller::new(&parameters);
    let dynamics = Dynamics::new(&parameters);
    let mut sensors = Sensors::new(&parameters);
    
    // Create time array
    let dt = parameters.sample;
    let end_time = 300.0;
    let time_steps = (end_time / dt) as usize + 1;
    let time: Vec<f64> = (0..time_steps).map(|i| i as f64 * dt).collect();
    
    // Initialize state
    let mut x = na::Vector7::zeros();
    x[0] = 1.0; // Initial quaternion [1, 0, 0, 0]
    
    // Initialize reference
    let mut r = na::Vector7::zeros();
    r[0] = 0.5;
    r[1] = 0.5;
    r[2] = -0.5;
    r[3] = -0.5;
    
    // Normalize reference quaternion
    let quat_norm = na::Vector4::new(r[0], r[1], r[2], r[3]).norm();
    r[0] /= quat_norm;
    r[1] /= quat_norm;
    r[2] /= quat_norm;
    r[3] /= quat_norm;
    
    // Initialize plots
    let mut plots = HashMap::new();
    
    // Quaternion plots
    plots.insert("q1_r".to_string(), PlotVariable { 
        color: rgb(255, 0, 0), unit: "".to_string(), index: 1, value: 0.0, disabled: false 
    });
    plots.insert("q1_hat".to_string(), PlotVariable { 
        color: rgb(0, 255, 0), unit: "".to_string(), index: 1, value: 0.0, disabled: false 
    });
    plots.insert("q1".to_string(), PlotVariable { 
        color: rgb(0, 0, 255), unit: "".to_string(), index: 1, value: 0.0, disabled: false 
    });
    
    plots.insert("q2_r".to_string(), PlotVariable { 
        color: rgb(255, 0, 0), unit: "".to_string(), index: 2, value: 0.0, disabled: false 
    });
    plots.insert("q2_hat".to_string(), PlotVariable { 
        color: rgb(0, 255, 0), unit: "".to_string(), index: 2, value: 0.0, disabled: false 
    });
    plots.insert("q2".to_string(), PlotVariable { 
        color: rgb(0, 0, 255), unit: "".to_string(), index: 2, value: 0.0, disabled: false 
    });
    
    plots.insert("q3_r".to_string(), PlotVariable { 
        color: rgb(255, 0, 0), unit: "".to_string(), index: 3, value: 0.0, disabled: false 
    });
    plots.insert("q3_hat".to_string(), PlotVariable { 
        color: rgb(0, 255, 0), unit: "".to_string(), index: 3, value: 0.0, disabled: false 
    });
    plots.insert("q3".to_string(), PlotVariable { 
        color: rgb(0, 0, 255), unit: "".to_string(), index: 3, value: 0.0, disabled: false 
    });
    
    plots.insert("q4_r".to_string(), PlotVariable { 
        color: rgb(255, 0, 0), unit: "".to_string(), index: 4, value: 0.0, disabled: false 
    });
    plots.insert("q4_hat".to_string(), PlotVariable { 
        color: rgb(0, 255, 0), unit: "".to_string(), index: 4, value: 0.0, disabled: false 
    });
    plots.insert("q4".to_string(), PlotVariable { 
        color: rgb(0, 0, 255), unit: "".to_string(), index: 4, value: 0.0, disabled: false 
    });
    
    // Angular velocity plots
    plots.insert("Wx_r".to_string(), PlotVariable { 
        color: rgb(255, 0, 0), unit: "rad/sec".to_string(), index: 5, value: 0.0, disabled: false 
    });
    plots.insert("Wx".to_string(), PlotVariable { 
        color: rgb(0, 0, 255), unit: "rad/sec".to_string(), index: 5, value: 0.0, disabled: false 
    });
    
    plots.insert("Wy_r".to_string(), PlotVariable { 
        color: rgb(255, 0, 0), unit: "rad/sec".to_string(), index: 6, value: 0.0, disabled: false 
    });
    plots.insert("Wy".to_string(), PlotVariable { 
        color: rgb(0, 0, 255), unit: "rad/sec".to_string(), index: 6, value: 0.0, disabled: false 
    });
    
    plots.insert("Wz_r".to_string(), PlotVariable { 
        color: rgb(255, 0, 0), unit: "rad/sec".to_string(), index: 7, value: 0.0, disabled: false 
    });
    plots.insert("Wz".to_string(), PlotVariable { 
        color: rgb(0, 0, 255), unit: "rad/sec".to_string(), index: 7, value: 0.0, disabled: false 
    });
    
    // Torque plots
    plots.insert("Tx".to_string(), PlotVariable { 
        color: rgb(255, 0, 0), unit: "torque".to_string(), index: 8, value: 0.0, disabled: false 
    });
    plots.insert("Ty".to_string(), PlotVariable { 
        color: rgb(0, 255, 0), unit: "torque".to_string(), index: 8, value: 0.0, disabled: false 
    });
    plots.insert("Tz".to_string(), PlotVariable { 
        color: rgb(0, 0, 255), unit: "Newton-meters".to_string(), index: 8, value: 0.0, disabled: false 
    });
    
    // Reaction wheel plots
    plots.insert("RW1".to_string(), PlotVariable { 
        color: rgb(0, 0, 255), unit: "RPM".to_string(), index: 9, value: 0.0, disabled: false 
    });
    plots.insert("RW2".to_string(), PlotVariable { 
        color: rgb(0, 255, 0), unit: "RPM".to_string(), index: 9, value: 0.0, disabled: false 
    });
    plots.insert("RW3".to_string(), PlotVariable { 
        color: rgb(255, 0, 0), unit: "RPM".to_string(), index: 9, value: 0.0, disabled: false 
    });
    plots.insert("RW4".to_string(), PlotVariable { 
        color: rgb(0, 255, 255), unit: "RPM".to_string(), index: 9, value: 0.0, disabled: false 
    });
    
    // Initialize plotter
    let mut plotter = Plotter::new(plots.clone(), &parameters);
    
    // Initialize control input
    let mut u = na::Vector3::zeros();
    
    // Main simulation loop
    for i in 0..time.len() {
        // Change reference every 20 seconds
        if (time[i] % 20.0).abs() < dt/2.0 {
            // Generate random reference quaternion
            let mut rng = rand::thread_rng();
            let random_quat = na::Vector4::<f64>::new(
                rand::random::<f64>(),
                rand::random::<f64>(),
                rand::random::<f64>(),
                rand::random::<f64>()
            );
            
            // Normalize
            let quat_norm = random_quat.norm();
            r[0] = random_quat[0] / quat_norm;
            r[1] = random_quat[1] / quat_norm;
            r[2] = random_quat[2] / quat_norm;
            r[3] = random_quat[3] / quat_norm;
        }
        
        // Get sensor measurements
        let measurements = sensors.update(&x, 0.0, time[i], false);
        
        // Update observer
        let x_hat = observer.update(&u, &measurements);
        
        // Update controller
        u = controller.update(&x, &r);
        
        // Convert torque to RPM
        let rpm = quaternion::tau2rpm(&u);
        
        // Convert torque to PWM
        let _pwm = quaternion::tau2pwm(&u);
        
        // Update dynamics
        x = dynamics.update(&x, &u, dt);
        
        // Extract quaternion for animation
        let y = na::Vector4::new(x[0], x[1], x[2], x[3]);
        
        // Update animation periodically (every 0.1 seconds)
        if (time[i] % 0.1).abs() < dt/2.0 {
            animation.update(&y);
        }
        
        // Update plot data
        plots.get_mut("q1").unwrap().value = x[0];
        plots.get_mut("q1_r").unwrap().value = r[0];
        plots.get_mut("q1_hat").unwrap().value = x_hat[0];
        
        plots.get_mut("q2").unwrap().value = x[1];
        plots.get_mut("q2_r").unwrap().value = r[1];
        plots.get_mut("q2_hat").unwrap().value = x_hat[1];
        
        plots.get_mut("q3").unwrap().value = x[2];
        plots.get_mut("q3_r").unwrap().value = r[2];
        plots.get_mut("q3_hat").unwrap().value = x_hat[2];
        
        plots.get_mut("q4").unwrap().value = x[3];
        plots.get_mut("q4_r").unwrap().value = r[3];
        plots.get_mut("q4_hat").unwrap().value = x_hat[3];
        
        plots.get_mut("Wx").unwrap().value = x[4];
        plots.get_mut("Wx_r").unwrap().value = r[4];
        
        plots.get_mut("Wy").unwrap().value = x[5];
        plots.get_mut("Wy_r").unwrap().value = r[5];
        
        plots.get_mut("Wz").unwrap().value = x[6];
        plots.get_mut("Wz_r").unwrap().value = r[6];
        
        plots.get_mut("Tx").unwrap().value = u[0];
        plots.get_mut("Ty").unwrap().value = u[1];
        plots.get_mut("Tz").unwrap().value = u[2];
        
        plots.get_mut("RW1").unwrap().value = rpm[0];
        plots.get_mut("RW2").unwrap().value = rpm[1];
        plots.get_mut("RW3").unwrap().value = rpm[2];
        plots.get_mut("RW4").unwrap().value = rpm[3];
        
        // Update time in parameters
        let mut params = parameters;
        params.time += params.sample;
        
        // Update plotter
        plotter.update(&plots, &params);
        
        // Add a small delay to not consume 100% CPU
        thread::sleep(Duration::from_millis(1));
    }
}