#include "core_HelperFunctions.hpp"

using namespace Param;

HelperFunctions::WMMCoeffs::WMMCoeffs() {
    // 1. Set Model Parameters (WMM-2025 Defaults)
    epoch = 2025.0;      // Model Epoch
    a = 6371200.0;       // Reference Radius (meters)

    // 2. Resize Vectors (Degree 12 Model -> Size 13x13)
    int size = MAX_DEGREE + 1; // 12 + 1 = 13
    g.resize(size, std::vector<Scalar>(size, 0.0));
    h.resize(size, std::vector<Scalar>(size, 0.0));
    dg.resize(size, std::vector<Scalar>(size, 0.0));
    dh.resize(size, std::vector<Scalar>(size, 0.0));

    // 3. Populate Coefficients [n][m]
    // Columns: n, m, g, h, dg, dh

    // Degree 1
    g[1][0] = -29351.8;   h[1][0] = 0.0;        dg[1][0] = 12.0;      dh[1][0] = 0.0;
    g[1][1] = -1410.8;    h[1][1] = 4545.4;     dg[1][1] = 9.7;       dh[1][1] = -21.5;

    // Degree 2
    g[2][0] = -2556.6;    h[2][0] = 0.0;        dg[2][0] = -11.6;     dh[2][0] = 0.0;
    g[2][1] = 2951.1;     h[2][1] = -3133.6;    dg[2][1] = -5.2;      dh[2][1] = -27.7;
    g[2][2] = 1649.3;     h[2][2] = -815.1;     dg[2][2] = -8.0;      dh[2][2] = -12.1;

    // Degree 3
    g[3][0] = 1361.0;     h[3][0] = 0.0;        dg[3][0] = -1.3;      dh[3][0] = 0.0;
    g[3][1] = -2404.1;    h[3][1] = -56.6;      dg[3][1] = -4.2;      dh[3][1] = 4.0;
    g[3][2] = 1243.8;     h[3][2] = 237.5;      dg[3][2] = 0.4;       dh[3][2] = -0.3;
    g[3][3] = 453.6;      h[3][3] = -549.5;     dg[3][3] = -15.6;     dh[3][3] = -4.1;

    // Degree 4
    g[4][0] = 895.0;      h[4][0] = 0.0;        dg[4][0] = -1.6;      dh[4][0] = 0.0;
    g[4][1] = 799.5;      h[4][1] = 278.6;      dg[4][1] = -2.4;      dh[4][1] = -1.1;
    g[4][2] = 55.7;       h[4][2] = -133.9;     dg[4][2] = -6.0;      dh[4][2] = 4.1;
    g[4][3] = -281.1;     h[4][3] = 212.0;      dg[4][3] = 5.6;       dh[4][3] = 1.6;
    g[4][4] = 12.1;       h[4][4] = -375.6;     dg[4][4] = -7.0;      dh[4][4] = -4.4;

    // Degree 5
    g[5][0] = -233.2;     h[5][0] = 0.0;        dg[5][0] = 0.6;       dh[5][0] = 0.0;
    g[5][1] = 368.9;      h[5][1] = 45.4;       dg[5][1] = 1.4;       dh[5][1] = -0.5;
    g[5][2] = 187.2;      h[5][2] = 220.2;      dg[5][2] = 0.0;       dh[5][2] = 2.2;
    g[5][3] = -138.7;     h[5][3] = -122.9;     dg[5][3] = 0.6;       dh[5][3] = 0.4;
    g[5][4] = -142.0;     h[5][4] = 43.0;       dg[5][4] = 2.2;       dh[5][4] = 1.7;
    g[5][5] = 20.9;       h[5][5] = 106.1;      dg[5][5] = 0.9;       dh[5][5] = 1.9;

    // Degree 6
    g[6][0] = 64.4;       h[6][0] = 0.0;        dg[6][0] = -0.2;      dh[6][0] = 0.0;
    g[6][1] = 63.8;       h[6][1] = -18.4;      dg[6][1] = -0.4;      dh[6][1] = 0.3;
    g[6][2] = 76.9;       h[6][2] = 16.8;       dg[6][2] = 0.9;       dh[6][2] = -1.6;
    g[6][3] = -115.7;     h[6][3] = 48.8;       dg[6][3] = 1.2;       dh[6][3] = -0.4;
    g[6][4] = -40.9;      h[6][4] = -59.8;      dg[6][4] = -0.9;      dh[6][4] = 0.9;
    g[6][5] = 14.9;       h[6][5] = 10.9;       dg[6][5] = 0.3;       dh[6][5] = 0.7;
    g[6][6] = -60.7;      h[6][6] = 72.7;       dg[6][6] = 0.9;       dh[6][6] = 0.9;

    // Degree 7
    g[7][0] = 79.5;       h[7][0] = 0.0;        dg[7][0] = -0.0;      dh[7][0] = 0.0;
    g[7][1] = -77.0;      h[7][1] = -48.9;      dg[7][1] = -0.1;      dh[7][1] = 0.6;
    g[7][2] = -8.8;       h[7][2] = -14.4;      dg[7][2] = -0.1;      dh[7][2] = 0.5;
    g[7][3] = 59.3;       h[7][3] = -1.0;       dg[7][3] = 0.5;       dh[7][3] = -0.8;
    g[7][4] = 15.8;       h[7][4] = 23.4;       dg[7][4] = -0.1;      dh[7][4] = 0.0;
    g[7][5] = 2.5;        h[7][5] = -7.4;       dg[7][5] = -0.8;      dh[7][5] = -1.0;
    g[7][6] = -11.1;      h[7][6] = -25.1;      dg[7][6] = -0.8;      dh[7][6] = 0.6;
    g[7][7] = 14.2;       h[7][7] = -2.3;       dg[7][7] = 0.8;       dh[7][7] = -0.2;

    // Degree 8
    g[8][0] = 23.2;       h[8][0] = 0.0;        dg[8][0] = -0.1;      dh[8][0] = 0.0;
    g[8][1] = 10.8;       h[8][1] = 7.1;        dg[8][1] = 0.2;       dh[8][1] = -0.2;
    g[8][2] = -17.5;      h[8][2] = -12.6;      dg[8][2] = 0.0;       dh[8][2] = 0.5;
    g[8][3] = 2.0;        h[8][3] = 11.4;       dg[8][3] = 0.5;       dh[8][3] = -0.4;
    g[8][4] = -21.7;      h[8][4] = -9.7;       dg[8][4] = -0.1;      dh[8][4] = 0.4;
    g[8][5] = 16.9;       h[8][5] = 12.7;       dg[8][5] = 0.3;       dh[8][5] = -0.5;
    g[8][6] = 15.0;       h[8][6] = 0.7;        dg[8][6] = 0.2;       dh[8][6] = -0.6;
    g[8][7] = -16.8;      h[8][7] = -5.2;       dg[8][7] = -0.0;      dh[8][7] = 0.3;
    g[8][8] = 0.9;        h[8][8] = 3.9;        dg[8][8] = 0.2;       dh[8][8] = 0.2;

    // Degree 9
    g[9][0] = 4.6;        h[9][0] = 0.0;        dg[9][0] = -0.0;      dh[9][0] = 0.0;
    g[9][1] = 7.8;        h[9][1] = -24.8;      dg[9][1] = -0.1;      dh[9][1] = -0.3;
    g[9][2] = 3.0;        h[9][2] = 12.2;       dg[9][2] = 0.1;       dh[9][2] = 0.3;
    g[9][3] = -0.2;       h[9][3] = 8.3;        dg[9][3] = 0.3;       dh[9][3] = -0.3;
    g[9][4] = -2.5;       h[9][4] = -3.3;       dg[9][4] = -0.3;      dh[9][4] = 0.3;
    g[9][5] = -13.1;      h[9][5] = -5.2;       dg[9][5] = 0.0;       dh[9][5] = 0.2;
    g[9][6] = 2.4;        h[9][6] = 7.2;        dg[9][6] = 0.3;       dh[9][6] = -0.1;
    g[9][7] = 8.6;        h[9][7] = -0.6;       dg[9][7] = -0.1;      dh[9][7] = -0.2;
    g[9][8] = -8.7;       h[9][8] = 0.8;        dg[9][8] = 0.1;       dh[9][8] = 0.4;
    g[9][9] = -12.9;      h[9][9] = 10.0;       dg[9][9] = -0.1;      dh[9][9] = 0.1;

    // Degree 10
    g[10][0] = -1.3;      h[10][0] = 0.0;       dg[10][0] = 0.1;      dh[10][0] = 0.0;
    g[10][1] = -6.4;      h[10][1] = 3.3;       dg[10][1] = 0.0;      dh[10][1] = 0.0;
    g[10][2] = 0.2;       h[10][2] = 0.0;       dg[10][2] = 0.1;      dh[10][2] = -0.0;
    g[10][3] = 2.0;       h[10][3] = 2.4;       dg[10][3] = 0.1;      dh[10][3] = -0.2;
    g[10][4] = -1.0;      h[10][4] = 5.3;       dg[10][4] = -0.0;     dh[10][4] = 0.1;
    g[10][5] = -0.6;      h[10][5] = -9.1;      dg[10][5] = -0.3;     dh[10][5] = -0.1;
    g[10][6] = -0.9;      h[10][6] = 0.4;       dg[10][6] = 0.0;      dh[10][6] = 0.1;
    g[10][7] = 1.5;       h[10][7] = -4.2;      dg[10][7] = -0.1;     dh[10][7] = 0.0;
    g[10][8] = 0.9;       h[10][8] = -3.8;      dg[10][8] = -0.1;     dh[10][8] = -0.1;
    g[10][9] = -2.7;      h[10][9] = 0.9;       dg[10][9] = -0.0;     dh[10][9] = 0.2;
    g[10][10] = -3.9;     h[10][10] = -9.1;     dg[10][10] = -0.0;    dh[10][10] = -0.0;

    // Degree 11
    g[11][0] = 2.9;       h[11][0] = 0.0;       dg[11][0] = 0.0;      dh[11][0] = 0.0;
    g[11][1] = -1.5;      h[11][1] = 0.0;       dg[11][1] = -0.0;     dh[11][1] = -0.0;
    g[11][2] = -2.5;      h[11][2] = 2.9;       dg[11][2] = 0.0;      dh[11][2] = 0.1;
    g[11][3] = 2.4;       h[11][3] = -0.6;      dg[11][3] = 0.0;      dh[11][3] = -0.0;
    g[11][4] = -0.6;      h[11][4] = 0.2;       dg[11][4] = 0.0;      dh[11][4] = 0.1;
    g[11][5] = -0.1;      h[11][5] = 0.5;       dg[11][5] = -0.1;     dh[11][5] = -0.0;
    g[11][6] = -0.6;      h[11][6] = -0.3;      dg[11][6] = 0.0;      dh[11][6] = -0.0;
    g[11][7] = -0.1;      h[11][7] = -1.2;      dg[11][7] = -0.0;     dh[11][7] = 0.1;
    g[11][8] = 1.1;       h[11][8] = -1.7;      dg[11][8] = -0.1;     dh[11][8] = -0.0;
    g[11][9] = -1.0;      h[11][9] = -2.9;      dg[11][9] = -0.1;     dh[11][9] = 0.0;
    g[11][10] = -0.2;     h[11][10] = -1.8;     dg[11][10] = -0.1;    dh[11][10] = 0.0;
    g[11][11] = 2.6;      h[11][11] = -2.3;     dg[11][11] = -0.1;    dh[11][11] = 0.0;

    // Degree 12
    g[12][0] = -2.0;      h[12][0] = 0.0;       dg[12][0] = 0.0;      dh[12][0] = 0.0;
    g[12][1] = -0.2;      h[12][1] = -1.3;      dg[12][1] = 0.0;      dh[12][1] = -0.0;
    g[12][2] = 0.3;       h[12][2] = 0.7;       dg[12][2] = -0.0;     dh[12][2] = 0.0;
    g[12][3] = 1.2;       h[12][3] = 1.0;       dg[12][3] = -0.0;     dh[12][3] = -0.1;
    g[12][4] = -1.3;      h[12][4] = -1.4;      dg[12][4] = -0.0;     dh[12][4] = 0.1;
    g[12][5] = 0.6;       h[12][5] = -0.0;      dg[12][5] = -0.0;     dh[12][5] = -0.0;
    g[12][6] = 0.6;       h[12][6] = 0.6;       dg[12][6] = 0.1;      dh[12][6] = -0.0;
    g[12][7] = 0.5;       h[12][7] = -0.1;      dg[12][7] = -0.0;     dh[12][7] = -0.0;
    g[12][8] = -0.1;      h[12][8] = 0.8;       dg[12][8] = 0.0;      dh[12][8] = 0.0;
    g[12][9] = -0.4;      h[12][9] = 0.1;       dg[12][9] = 0.0;      dh[12][9] = -0.0;
    g[12][10] = -0.2;     h[12][10] = -1.0;     dg[12][10] = -0.1;    dh[12][10] = -0.0;
    g[12][11] = -1.3;     h[12][11] = 0.1;      dg[12][11] = -0.0;    dh[12][11] = 0.0;
    g[12][12] = -0.7;     h[12][12] = 0.2;      dg[12][12] = -0.1;    dh[12][12] = -0.1;
}

HelperFunctions::HelperFunctions() {

}

HelperFunctions::Scalar HelperFunctions::julianDate(Scalar unixTimestamp) {
    return (unixTimestamp / 86400.0) + 2440587.5;
}

HelperFunctions::Vector3 HelperFunctions::earth2sun(Scalar jd) {
    // MATLAB Logic
    Scalar T = (jd - 2451545.0) / 36525.0;

    // Mean longitude & anomaly
    Scalar L_0 = 280.46646 + 36000.76983*T + 0.0003032*T*T;
    Scalar M   = 357.52911 + 35999.05029*T - 0.0001537*T*T;

    // Eccentricity
    Scalar e_earth = 0.016708634 - 0.000042037*T - 0.0000001267*T*T;

    // Equation of Center
    Scalar M_rad = M * Shared::deg2rad;
    
    Scalar C = (1.914602 - 0.004817*T - 0.000014*T*T) * std::sin(M_rad) +
               (0.019993 - 0.000101*T) * std::sin(2.0*M_rad) +
               0.000289 * std::sin(3.0*M_rad);

    Scalar true_long = L_0 + C;
    Scalar true_anom = M + C;
    
    // Distance (AU)
    Scalar numer = 0.999722; // Interpolated value from MATLAB
    Scalar true_anom_rad = true_anom * Shared::deg2rad;
    Scalar D = numer * (1.0 - e_earth*e_earth) / (1.0 + e_earth * std::cos(true_anom_rad));

    // Obliquity
    Scalar eps = 23.439291 - (46.8150/3600.0)*T - (0.00059/3600.0)*T*T + (0.001813/3600.0)*T*T*T;

    // Coordinates
    Scalar eps_rad = eps * Shared::deg2rad;
    Scalar tl_rad = true_long * Shared::deg2rad;

    Scalar x = D * std::cos(tl_rad);
    Scalar y = D * std::cos(eps_rad) * std::sin(tl_rad);
    Scalar z = D * std::sin(eps_rad) * std::sin(tl_rad);

    Vector3 R; 
    R(0) = x; R(1) = y; R(2) = z;
    return R.normalized();
}

Param::Vector4 HelperFunctions::quatFromTwoVectors(const Vector3& v_from, const Vector3& v_to) {
    Vector3 v1 = v_from / (v_from.norm() + 1e-12);
    Vector3 v2 = v_to   / (v_to.norm()   + 1e-12);

    Scalar dotProd = v1.dot(v2);
    Vector3 crossProd = v1.cross(v2);

    Vector4 q;

    if (dotProd < -0.999999) {
        // Opposite vectors
        Vector3 orth = Vector3::UnitX();
        if (std::abs(v1(0)) > 0.9) {
            orth = Vector3::UnitY();
        }
        Vector3 rotAxis = v1.cross(orth);
        rotAxis = rotAxis / (rotAxis.norm() + 1e-12);
        
        q(0) = 0.0; q(1) = rotAxis(0); q(2) = rotAxis(1); q(3) = rotAxis(2);
    } else {
        q(0) = 1.0 + dotProd;
        q(1) = crossProd(0);
        q(2) = crossProd(1);
        q(3) = crossProd(2);
        q = q / (q.norm() + 1e-12);
    }
    return q; 
}

Param::Vector4 HelperFunctions::quatMultiply(const Quat& q1, const Quat& q2) {
    Scalar w1 = q1(0); 
    Vector3 v1; v1(0) = q1(1); v1(1) = q1(2); v1(2) = q1(3);
    
    Scalar w2 = q2(0); 
    Vector3 v2; v2(0) = q2(1); v2(1) = q2(2); v2(2) = q2(3);

    Scalar w = w1*w2 - v1.dot(v2);
    Vector3 v = v1*w2 + v2*w1 + v1.cross(v2);

    Vector4 q;
    q(0) = w; q(1) = v(0); q(2) = v(1); q(3) = v(2);
    return q;
}

Param::Vector4 HelperFunctions::quatconj(const Quat& q) {
    Vector4 qc;
    qc(0) = q(0); qc(1) = -q(1); qc(2) = -q(2); qc(3) = -q(3);
    return qc;
}

HelperFunctions::Vector3 HelperFunctions::quatRotate(const Quat& q, const Vector3& v) {
    Vector4 qv;
    qv(0) = 0.0; qv(1) = v(0); qv(2) = v(1); qv(3) = v(2);

    Vector4 temp = quatMultiply(q, qv);
    Vector4 v_rot_q = quatMultiply(temp, quatconj(q));

    Vector3 result;
    result(0) = v_rot_q(1); result(1) = v_rot_q(2); result(2) = v_rot_q(3);
    return result;
}

HelperFunctions::Matrix3 HelperFunctions::dcmeci2ecef(Scalar jd) {
    Scalar GMST = std::fmod(280.46061837 + 360.98564736629*(jd - 2451545.0), 360.0);
    // Handle negative mod result in C++
    if (GMST < 0) GMST += 360.0;

    Scalar theta = GMST * Shared::deg2rad;
    Scalar c = std::cos(theta);
    Scalar s = std::sin(theta);
    
    Matrix3 C;
    C(0,0) = c;  C(0,1) = s;  C(0,2) = 0;
    C(1,0) = -s; C(1,1) = c;  C(1,2) = 0;
    C(2,0) = 0;  C(2,1) = 0;  C(2,2) = 1;
    return C;
}

HelperFunctions::Vector3 HelperFunctions::eci2ecef(const Vector3& r_eci, Scalar jd) {
    Matrix3 C = dcmeci2ecef(jd);
    return C * r_eci;
}

HelperFunctions::Ecef2llaOutput HelperFunctions::ecef2lla(const Vector3& r) {
    // MATLAB Logic
    Scalar x = r(0), y = r(1), z = r(2);
    Scalar a = 6378137.0;
    Scalar f = 1.0/298.257223563;
    Scalar e2 = f*(2.0-f);
    
    Scalar lon = std::atan2(y, x);
    Scalar rho = std::sqrt(x*x + y*y);
    Scalar lat = std::atan2(z, rho*(1.0 - e2));
    Scalar lat_prev = 0.0;
    Scalar alt = 0.0;
    Scalar N = 0.0;
    
    // Iterative solution
    int iter = 0;
    while (std::abs(lat - lat_prev) > 1e-12 && iter < 100) {
        lat_prev = lat;
        Scalar sin_lat = std::sin(lat);
        N = a / std::sqrt(1.0 - e2*sin_lat*sin_lat);
        alt = rho / std::cos(lat) - N;
        lat = std::atan2(z, rho*(1.0 - e2*e2*N/(N+alt)));
        iter++;
    }
    // Final calc
    Scalar sin_lat = std::sin(lat);
    N = a / std::sqrt(1.0 - e2*sin_lat*sin_lat);
    alt = rho / std::cos(lat) - N;

    return Ecef2llaOutput{lat, lon, alt};
}

HelperFunctions::Matrix3 HelperFunctions::dcmecef2ned(Scalar lat, Scalar lon) {
    Scalar sL = std::sin(lat); 
    Scalar cL = std::cos(lat);
    Scalar s_lam = std::sin(lon); 
    Scalar c_lam = std::cos(lon);

    Matrix3 C;
    C(0,0) = -sL*c_lam; C(0,1) = -sL*s_lam; C(0,2) = cL;
    C(1,0) = -s_lam;    C(1,1) = c_lam;     C(1,2) = 0.0;
    C(2,0) = -cL*c_lam; C(2,1) = -cL*s_lam; C(2,2) = -sL;
    return C;
}

HelperFunctions::Vector3 HelperFunctions::wrldmagm(Scalar lat, Scalar lon, Scalar alt, Scalar decYear) {
    // 1. Setup Constants
    Scalar a = wmm.a;
    Scalar f = 1.0/298.257223563;
    Scalar e2 = f*(2.0 - f);
    
    // 2. Geodetic to Geocentric
    Scalar sin_lat = std::sin(lat);
    Scalar cos_lat = std::cos(lat);
    Scalar N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
    
    Scalar X = (N + alt) * cos_lat;
    Scalar Z = (N * (1.0 - e2) + alt) * sin_lat;
    Scalar r = std::sqrt(X*X + Z*Z);
    Scalar phi_gc = std::atan((1.0 - e2) * std::tan(lat));
    
    Scalar theta = (Shared::PI / 2.0) - phi_gc; // Colatitude
    
    // 3. Precompute sin/cos(m*lon)
    int Nmax = WMMCoeffs::MAX_DEGREE;
    std::vector<Scalar> cosmlon(Nmax + 1);
    std::vector<Scalar> sinmlon(Nmax + 1);
    for (int m = 0; m <= Nmax; ++m) {
        cosmlon[m] = std::cos(m * lon);
        sinmlon[m] = std::sin(m * lon);
    }
    
    // 4. Time Adjustment
    Scalar dt = decYear - wmm.epoch;
    std::vector<std::vector<Scalar>> g = wmm.g; 
    std::vector<std::vector<Scalar>> h = wmm.h;
    
    for (int n = 1; n <= Nmax; ++n) {
        for (int m = 0; m <= n; ++m) {
            g[n][m] += wmm.dg[n][m] * dt;
            h[n][m] += wmm.dh[n][m] * dt;
            // Convert nT to Tesla
            g[n][m] *= 1e-9;
            h[n][m] *= 1e-9;
        }
    }

    // 5. Schmidt Semi-Normalized Polynomials
    std::vector<std::vector<Scalar>> P(Nmax + 2, std::vector<Scalar>(Nmax + 2, 0.0));
    P[0][0] = 1.0; 

    for (int n = 1; n <= Nmax; ++n) {
        Scalar c_theta = std::cos(theta);
        Scalar s_theta = std::sin(theta);
        
        P[n][n] = (2.0*n - 1.0) * s_theta * P[n-1][n-1];
        P[n][n-1] = (2.0*n - 1.0) * c_theta * P[n-1][n-1];
        
        for (int m = 0; m <= n - 2; ++m) {
            P[n][m] = ((2.0*n - 1.0)*c_theta*P[n-1][m] - (n + m - 1.0)*P[n-2][m]) / (Scalar)(n - m);
        }
    }
    
    // Schmidt Normalization
    for (int n = 0; n <= Nmax; ++n) {
        for (int m = 0; m <= n; ++m) {
            Scalar S_nm;
            if (m == 0) {
                S_nm = std::sqrt(2.0*n + 1.0);
            } else {
                Scalar fact_n_m = std::tgamma(n - m + 1); // (n-m)!
                Scalar fact_n_p_m = std::tgamma(n + m + 1); // (n+m)!
                S_nm = std::sqrt(2.0 * (2.0*n + 1.0) * fact_n_m / fact_n_p_m);
            }
            if(n==0 && m==0) S_nm = 1.0;
            P[n][m] *= S_nm;
        }
    }

    // 6. Accumulate Field
    Scalar Br = 0.0, Bt = 0.0, Bp = 0.0;
    Scalar a_over_r = a / r;
    
    for (int n = 1; n <= Nmax; ++n) {
        Scalar ar_pwr = std::pow(a_over_r, n + 2); // (a/r)^(n+2)
        
        for (int m = 0; m <= n; ++m) {
            Scalar V = g[n][m]*cosmlon[m] + h[n][m]*sinmlon[m];
            Scalar W = -g[n][m]*sinmlon[m] + h[n][m]*cosmlon[m];
            
            Scalar dP = diff_legendre(P, n, m, theta);
            
            Br -= (n + 1.0) * ar_pwr * V * P[n][m];
            Bt -= ar_pwr * V * dP;
            
            if (std::abs(std::sin(theta)) > 1e-10) {
                Bp -= ar_pwr * (m * P[n][m] / std::sin(theta)) * W;
            } else {
                Bp = 0.0;
            }
        }
    }
    
    // 7. Spherical -> NED
    Vector3 out;
    out(0) = -Bt; out(1) = Bp; out(2) = -Br;
    
    return out;
}

HelperFunctions::Scalar HelperFunctions::diff_legendre(const std::vector<std::vector<Scalar>>& P, 
                                                       int n, int m, Scalar theta) 
{
    if (n == 0) return 0.0;
    
    Scalar top = n * std::cos(theta) * P[n][m] - (n + m) * P[n-1][m];
    return top / std::sin(theta);
}

HelperFunctions::Quat HelperFunctions::quatFromTwoVectorPairs(
    const Vector3& b1, const Vector3& t1, 
    const Vector3& b2, const Vector3& t2) 
{
    // ===== Build inertial triad =====
    Vector3 i1 = t1.normalized();
    Vector3 i3_raw = t1.cross(t2);
    Vector3 i3, i2;
    
    if (i3_raw.norm() < 1e-9) {
        // t1 and t2 are parallel - pick arbitrary perpendicular axis
        Vector3 orth = (std::abs(i1(0)) < 0.9) ? Vector3::UnitX() : Vector3::UnitY();
        i2 = orth.cross(i1).normalized();
        i3 = i1.cross(i2);
    } else {
        i3 = i3_raw.normalized();
        i2 = i3.cross(i1);
    }

    // ===== Build body triad =====
    Vector3 b1_hat = b1.normalized();
    Vector3 b3_raw = b1.cross(b2);
    Vector3 b3, b2_hat;
    
    if (b3_raw.norm() < 1e-9) {
        // b1 and b2 are parallel - pick arbitrary perpendicular axis
        Vector3 orth = (std::abs(b1_hat(0)) < 0.9) ? Vector3::UnitX() : Vector3::UnitY();
        b2_hat = orth.cross(b1_hat).normalized();
        b3 = b1_hat.cross(b2_hat);
    } else {
        b3 = b3_raw.normalized();
        b2_hat = b3.cross(b1_hat);
    }

    // ===== DCM from body to inertial =====
    Matrix3 R_I; 
    R_I(0,0) = i1(0); R_I(0,1) = i2(0); R_I(0,2) = i3(0);
    R_I(1,0) = i1(1); R_I(1,1) = i2(1); R_I(1,2) = i3(1);
    R_I(2,0) = i1(2); R_I(2,1) = i2(2); R_I(2,2) = i3(2);
           
    Matrix3 R_B;
    R_B(0,0) = b1_hat(0); R_B(0,1) = b2_hat(0); R_B(0,2) = b3(0);
    R_B(1,0) = b1_hat(1); R_B(1,1) = b2_hat(1); R_B(1,2) = b3(1);
    R_B(2,0) = b1_hat(2); R_B(2,1) = b2_hat(2); R_B(2,2) = b3(2);
    
    Matrix3 C_BI = R_I * R_B.transpose();

    // ===== Convert DCM to quaternion (Shepperd's method) =====
    Quat q;
    Scalar trace = C_BI.trace();
    
    if (trace > 0.0) {
        Scalar S = std::sqrt(trace + 1.0) * 2.0; // S = 4*qw
        q(0) = 0.25 * S;
        q(1) = (C_BI(2,1) - C_BI(1,2)) / S;
        q(2) = (C_BI(0,2) - C_BI(2,0)) / S;
        q(3) = (C_BI(1,0) - C_BI(0,1)) / S;
    } else if ((C_BI(0,0) > C_BI(1,1)) && (C_BI(0,0) > C_BI(2,2))) {
        Scalar S = std::sqrt(1.0 + C_BI(0,0) - C_BI(1,1) - C_BI(2,2)) * 2.0; // S = 4*qx
        q(0) = (C_BI(2,1) - C_BI(1,2)) / S;
        q(1) = 0.25 * S;
        q(2) = (C_BI(0,1) + C_BI(1,0)) / S;
        q(3) = (C_BI(0,2) + C_BI(2,0)) / S;
    } else if (C_BI(1,1) > C_BI(2,2)) {
        Scalar S = std::sqrt(1.0 + C_BI(1,1) - C_BI(0,0) - C_BI(2,2)) * 2.0; // S = 4*qy
        q(0) = (C_BI(0,2) - C_BI(2,0)) / S;
        q(1) = (C_BI(0,1) + C_BI(1,0)) / S;
        q(2) = 0.25 * S;
        q(3) = (C_BI(1,2) + C_BI(2,1)) / S;
    } else {
        Scalar S = std::sqrt(1.0 + C_BI(2,2) - C_BI(0,0) - C_BI(1,1)) * 2.0; // S = 4*qz
        q(0) = (C_BI(1,0) - C_BI(0,1)) / S;
        q(1) = (C_BI(0,2) + C_BI(2,0)) / S;
        q(2) = (C_BI(1,2) + C_BI(2,1)) / S;
        q(3) = 0.25 * S;
    }

    // Ensure unit quaternion
    q.normalize();
    
    return q; 
}
// DCM to Quaternion conversion (Shepperd's method)
HelperFunctions::Quat HelperFunctions::dcm2quat(const Matrix3& C) {
    Quat q;
    Scalar trace = C.trace();
    
    if (trace > 0.0) {
        Scalar S = std::sqrt(trace + 1.0) * 2.0; // S = 4*qw
        q(0) = 0.25 * S;
        q(1) = (C(2,1) - C(1,2)) / S;
        q(2) = (C(0,2) - C(2,0)) / S;
        q(3) = (C(1,0) - C(0,1)) / S;
    } else if ((C(0,0) > C(1,1)) && (C(0,0) > C(2,2))) {
        Scalar S = std::sqrt(1.0 + C(0,0) - C(1,1) - C(2,2)) * 2.0; // S = 4*qx
        q(0) = (C(2,1) - C(1,2)) / S;
        q(1) = 0.25 * S;
        q(2) = (C(0,1) + C(1,0)) / S;
        q(3) = (C(0,2) + C(2,0)) / S;
    } else if (C(1,1) > C(2,2)) {
        Scalar S = std::sqrt(1.0 + C(1,1) - C(0,0) - C(2,2)) * 2.0; // S = 4*qy
        q(0) = (C(0,2) - C(2,0)) / S;
        q(1) = (C(0,1) + C(1,0)) / S;
        q(2) = 0.25 * S;
        q(3) = (C(1,2) + C(2,1)) / S;
    } else {
        Scalar S = std::sqrt(1.0 + C(2,2) - C(0,0) - C(1,1)) * 2.0; // S = 4*qz
        q(0) = (C(1,0) - C(0,1)) / S;
        q(1) = (C(0,2) + C(2,0)) / S;
        q(2) = (C(1,2) + C(2,1)) / S;
        q(3) = 0.25 * S;
    }

    // Ensure unit quaternion
    q.normalize();
    
    return q;
}