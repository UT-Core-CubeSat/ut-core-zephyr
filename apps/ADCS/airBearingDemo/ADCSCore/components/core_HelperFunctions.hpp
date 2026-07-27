#ifndef CORE_HELPERFUNCTIONS_HPP
#define CORE_HELPERFUNCTIONS_HPP

#include "core_Parameters.hpp"

class HelperFunctions {
public: 

    // Type Aliases
    using StateVector = Param::Vector7;
    using Reference = Param::Vector10;
    using Scalar = Param::Real;
    using TimeReal = Param::TimeReal;
    using Vector3 = Param::Vector3;
    using Vector4 = Param::Vector4;
    using Quat = Param::Vector4;
    using Matrix3 = Param::Matrix3;

    // Constructor 
    HelperFunctions();

    // --- Core Math ---
    // Pass Scalars by value (more efficient than reference for doubles)
    Quat quatFromTwoVectors(const Vector3& v_from, const Vector3& v_to);
    Quat quatFromTwoVectorPairs(const Vector3& b1, const Vector3& t1, 
                                const Vector3& b2, const Vector3& t2);
    Param::TimeReal julianDate(Param::TimeReal unix_timestamp);
    Quat quatMultiply(const Quat& q1, const Quat& q2);
    Quat quatconj(const Quat& q);
    Vector3 quatRotate(const Quat& q, const Vector3& v);

    // --- Ephemeris & Frames ---
    // Fixed: Returns Vector3 (Position), not Matrix3
    Vector3 earth2sun(Param::TimeReal jd); 
    
    // Fixed: Takes Vector3 (Position), returns Vector3
    Vector3 eci2ecef(const Vector3& r_eci, Param::TimeReal jd); 
    Matrix3 dcmeci2ecef(Param::TimeReal jd);
    Matrix3 dcmecef2ned(Scalar lat, Scalar lon);
    Quat dcm2quat(const Matrix3& C);

    // --- Coordinates ---
    struct Ecef2llaOutput {
        Scalar lat;
        Scalar lon;
        Scalar alt;
    };
    Ecef2llaOutput ecef2lla(const Vector3& r);


    // WMM Coefficient Struct - Fixed-size arrays for embedded systems
    struct WMMCoeffs {
        Scalar epoch;
        Scalar a;
        static constexpr int MAX_DEGREE = 12;
        static constexpr int SIZE = MAX_DEGREE + 1;
        static constexpr int P_SIZE = MAX_DEGREE + 2;  // For Legendre polynomials

        Scalar g[SIZE][SIZE];
        Scalar h[SIZE][SIZE];
        Scalar dg[SIZE][SIZE];
        Scalar dh[SIZE][SIZE];

        WMMCoeffs(); // Constructor declaration
    };

    Vector3 wrldmagm(Scalar lat, Scalar lon, Scalar alt, Scalar decYear);

private: 
    // Private Members
    WMMCoeffs wmm;

    // Private Methods
    Scalar diff_legendre(Scalar P[][WMMCoeffs::P_SIZE],
                         int n, 
                         int m, 
                         Scalar c_theta,
                         Scalar s_theta);
};

#endif // CORE_HELPERFUNCTIONS_HPP