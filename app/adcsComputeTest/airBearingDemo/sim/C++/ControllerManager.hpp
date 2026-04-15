#ifndef CONTROLLERMANAGER_HPP
#define CONTROLLERMANAGER_HPP

#include "Parameters.hpp"
#include "ControllerNDI.hpp"
#include "ControllerBDot.hpp"

class ControllerManager {
public: 

    // Input / Output Structs 
    struct ControlOutput {
        Param::Vector7 tau;
        Param::Vector7 states_m;
    };


    // Constructor 
    ControllerManager();

    // Update Method
    ControlOutput update(const Param::Vector17& states_hat, 
                         const Param::Vector10& reference,
                         const Param::Vector29& measurements,
                         const Param::PointingMode mode);
private:
    // Controllers
    ControllerNDI NDI;
    ControllerBDot Bdot;
    Param::Real Ts;
    Param::PointingMode prev_mode;
};

#endif // CONTROLLERMANAGER_HPP