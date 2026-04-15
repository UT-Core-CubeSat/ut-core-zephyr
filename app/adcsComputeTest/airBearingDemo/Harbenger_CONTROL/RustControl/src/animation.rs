use nalgebra as na;
use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::scene::SceneNode;

pub struct Animation {
    window: Window,
    cube_faces: Vec<SceneNode>,
}

impl Animation {
    pub fn new() -> Self {
        // Create a new window for rendering
        let mut window = Window::new("CubeSat Simulation");
        
        // Set up lighting
        window.set_light(Light::StickToCamera);
        
        // Define cube dimensions
        let half_x = 0.5;
        let half_y = 0.5;
        let half_z = 1.5;
        
        // Define colors for each face
        let face_colors = vec![
            na::Point3::new(1.0, 0.0, 0.0),  // Red
            na::Point3::new(0.0, 1.0, 0.0),  // Green
            na::Point3::new(0.0, 0.0, 1.0),  // Blue
            na::Point3::new(1.0, 1.0, 0.0),  // Yellow
            na::Point3::new(1.0, 0.0, 1.0),  // Magenta
            na::Point3::new(0.0, 1.0, 1.0),  // Cyan
        ];
        
        // Create cube faces
        let mut cube_faces = Vec::new();
        
        // Create a separate cube for each face
        for i in 0..6 {
            let mut face = window.add_cube(2.0 * half_x, 2.0 * half_y, 2.0 * half_z);
            face.set_color(face_colors[i].x, face_colors[i].y, face_colors[i].z);
            face.set_alpha(0.8);
            cube_faces.push(face);
        }
        
        // Set initial view
        window.set_point_of_view(na::Point3::new(0.0, 0.0, 10.0));
        
        Animation {
            window,
            cube_faces,
        }
    }
    
    pub fn update(&mut self, q: &na::Vector4<f64>) {
        // Extract quaternion components
        let q0 = q[0];
        let q1 = q[1];
        let q2 = q[2];
        let q3 = q[3];
        
        // Create rotation matrix from quaternion
        let r = na::Matrix3::new(
            1.0 - 2.0*(q2*q2 + q3*q3), 2.0*(q1*q2 - q0*q3), 2.0*(q1*q3 + q0*q2),
            2.0*(q1*q2 + q0*q3), 1.0 - 2.0*(q1*q1 + q3*q3), 2.0*(q2*q3 - q0*q1),
            2.0*(q1*q3 - q0*q2), 2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2)
        );
        
        // Convert to rotation matrix format used by kiss3d
        let rotation = na::UnitQuaternion::from_quaternion(
            na::Quaternion::new(q0 as f32, q1 as f32, q2 as f32, q3 as f32)
        );
        
        // Update each face
        for face in &mut self.cube_faces {
            face.set_local_rotation(rotation);
        }
        
        // Render the scene
        self.window.render();
    }
}