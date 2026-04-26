#ifndef CORE_CONTROLLERMANAGER_HPP
#define CORE_CONTROLLERMANAGER_HPP

#include "core_Parameters.hpp"
#include "core_ControllerNDI.hpp"
#include "core_ControllerBDot.hpp"

class ControllerManager {
public: 

    // Input / Output Structs
    struct ControlOutput {
        Param::Vector4 wheel_rpm;   // [RPM] desired wheel speeds (0 in OFF/DETUMBLE)
        Param::Vector3 mtq_dipole;  // [A·m²] magnetorquer dipole (0 in OFF/MOTOR)
        Param::Vector4 mtq_coil_currents; // [-] boolean face enables: [Xpos_en, Xneg_en, Ypos_en, Yneg_en], 0 or 1
    };


    // Constructor 
    ControllerManager();

    // Update Method
    ControlOutput update(const Param::Vector11& states_hat, 
                         const Param::Vector10& reference,
                         const Param::Vector13& measurements,
                         const Param::PointingMode mode, const Param::Real dt);
private:
    // Controllers
    ControllerNDI NDI;
    ControllerBDot Bdot;
    Param::PointingMode prev_mode;
};

#endif // CORE_CONTROLLERMANAGER_HPP