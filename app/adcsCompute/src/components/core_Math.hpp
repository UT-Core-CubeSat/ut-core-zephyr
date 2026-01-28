#ifndef CORE_MATH_HPP
#define CORE_MATH_HPP

#include <cmath>
#include <algorithm>
#include <cstring>
#include <limits>
#include <initializer_list>

// ============================================================================
// LIGHTWEIGHT MATH LIBRARY FOR EMBEDDED SYSTEMS
// Replaces Eigen with fixed-size, stack-allocated types
// ============================================================================

namespace Math {

constexpr double PI = 3.14159265358979323846;
// Forward declarations
template<int N> struct Vec;
template<int R, int C> struct Mat;

// ============================================================================
// FIXED-SIZE VECTOR
// ============================================================================
template<int N>
struct Vec {
    double data[N];

    // Default constructor (zero-initialized)
    Vec() { std::memset(data, 0, sizeof(data)); }

    // Constructor from initializer list
    Vec(std::initializer_list<double> init) {
        std::memset(data, 0, sizeof(data));
        int i = 0;
        for (auto val : init) {
            if (i < N) data[i++] = val;
        }
    }

    // Scalar fill constructor
    explicit Vec(double val) {
        for (int i = 0; i < N; ++i) data[i] = val;
    }

    // Element access
    double& operator()(int i) { return data[i]; }
    const double& operator()(int i) const { return data[i]; }

    // Static constructors
    static Vec Zero() { return Vec(); }
    
    static Vec Constant(double val) { return Vec(val); }

    static Vec UnitX() { 
        static_assert(N >= 1, "Vector too small for UnitX");
        Vec v; v.data[0] = 1.0; return v; 
    }
    static Vec UnitY() { 
        static_assert(N >= 2, "Vector too small for UnitY");
        Vec v; v.data[1] = 1.0; return v; 
    }
    static Vec UnitZ() { 
        static_assert(N >= 3, "Vector too small for UnitZ");
        Vec v; v.data[2] = 1.0; return v; 
    }

    // Arithmetic operators
    Vec operator+(const Vec& other) const {
        Vec result;
        for (int i = 0; i < N; ++i) result.data[i] = data[i] + other.data[i];
        return result;
    }

    Vec operator-(const Vec& other) const {
        Vec result;
        for (int i = 0; i < N; ++i) result.data[i] = data[i] - other.data[i];
        return result;
    }

    Vec operator-() const {
        Vec result;
        for (int i = 0; i < N; ++i) result.data[i] = -data[i];
        return result;
    }

    Vec operator*(double s) const {
        Vec result;
        for (int i = 0; i < N; ++i) result.data[i] = data[i] * s;
        return result;
    }

    Vec operator/(double s) const {
        Vec result;
        for (int i = 0; i < N; ++i) result.data[i] = data[i] / s;
        return result;
    }

    Vec& operator+=(const Vec& other) {
        for (int i = 0; i < N; ++i) data[i] += other.data[i];
        return *this;
    }

    Vec& operator-=(const Vec& other) {
        for (int i = 0; i < N; ++i) data[i] -= other.data[i];
        return *this;
    }

    Vec& operator*=(double s) {
        for (int i = 0; i < N; ++i) data[i] *= s;
        return *this;
    }

    Vec& operator/=(double s) {
        for (int i = 0; i < N; ++i) data[i] /= s;
        return *this;
    }

    // Dot product
    double dot(const Vec& other) const {
        double sum = 0;
        for (int i = 0; i < N; ++i) sum += data[i] * other.data[i];
        return sum;
    }

    // Cross product (only for 3D vectors)
    template<int M = N>
    typename std::enable_if<M == 3, Vec<3>>::type cross(const Vec<3>& other) const {
        Vec<3> result;
        result.data[0] = data[1] * other.data[2] - data[2] * other.data[1];
        result.data[1] = data[2] * other.data[0] - data[0] * other.data[2];
        result.data[2] = data[0] * other.data[1] - data[1] * other.data[0];
        return result;
    }

    // Norm
    double norm() const { return std::sqrt(dot(*this)); }

    double squaredNorm() const { return dot(*this); }

    // Normalize in place
    void normalize() {
        double n = norm();
        if (n > 1e-15) {
            for (int i = 0; i < N; ++i) data[i] /= n;
        }
    }

    // Return normalized copy
    Vec normalized() const {
        Vec result = *this;
        result.normalize();
        return result;
    }

    // Element-wise operations
    Vec cwiseProduct(const Vec& other) const {
        Vec result;
        for (int i = 0; i < N; ++i) result.data[i] = data[i] * other.data[i];
        return result;
    }

    Vec cwiseMax(double val) const {
        Vec result;
        for (int i = 0; i < N; ++i) result.data[i] = std::max(data[i], val);
        return result;
    }

    Vec cwiseMax(const Vec& other) const {
        Vec result;
        for (int i = 0; i < N; ++i) result.data[i] = std::max(data[i], other.data[i]);
        return result;
    }

    Vec cwiseMin(double val) const {
        Vec result;
        for (int i = 0; i < N; ++i) result.data[i] = std::min(data[i], val);
        return result;
    }

    Vec cwiseMin(const Vec& other) const {
        Vec result;
        for (int i = 0; i < N; ++i) result.data[i] = std::min(data[i], other.data[i]);
        return result;
    }

    // Sum of all elements
    double sum() const {
        double s = 0;
        for (int i = 0; i < N; ++i) s += data[i];
        return s;
    }

    // Mean of all elements
    double mean() const { return sum() / N; }

    // Check if all zeros
    bool isZero(double tol = 1e-15) const {
        for (int i = 0; i < N; ++i) {
            if (std::abs(data[i]) > tol) return false;
        }
        return true;
    }

    // Check for NaN
    bool hasNaN() const {
        for (int i = 0; i < N; ++i) {
            if (std::isnan(data[i])) return true;
        }
        return false;
    }

    // Segment extraction - returns a new vector of size M starting at index Start
    template<int M>
    Vec<M> segment(int start) const {
        Vec<M> result;
        for (int i = 0; i < M; ++i) result.data[i] = data[start + i];
        return result;
    }

    // Set segment
    template<int M>
    void setSegment(int start, const Vec<M>& src) {
        for (int i = 0; i < M; ++i) data[start + i] = src.data[i];
    }

    // Head (first M elements)
    template<int M>
    Vec<M> head() const { return segment<M>(0); }

    // Tail (last M elements)
    template<int M>
    Vec<M> tail() const { return segment<M>(N - M); }


    // Transpose - converts column vector (N×1) to row vector (1×N)
    Mat<1, N> transpose() const {
        Mat<1, N> result;
        for (int i = 0; i < N; ++i) {
            result(0, i) = data[i];
        }
        return result;
    }
};

// Scalar * Vector
template<int N>
Vec<N> operator*(double s, const Vec<N>& v) { return v * s; }

// ============================================================================
// FIXED-SIZE MATRIX (Row-Major Storage)
// ============================================================================
template<int R, int C>
struct Mat {
    double data[R * C];

    // Default constructor (zero-initialized)
    Mat() { std::memset(data, 0, sizeof(data)); }

    // Element access (row, col)
    double& operator()(int r, int c) { return data[r * C + c]; }
    const double& operator()(int r, int c) const { return data[r * C + c]; }

    // Static constructors
    static Mat Zero() { return Mat(); }

    static Mat Identity() {
        static_assert(R == C, "Identity only for square matrices");
        Mat m;
        for (int i = 0; i < R; ++i) m(i, i) = 1.0;
        return m;
    }

    // Arithmetic operators
    Mat operator+(const Mat& other) const {
        Mat result;
        for (int i = 0; i < R * C; ++i) result.data[i] = data[i] + other.data[i];
        return result;
    }

    Mat operator-(const Mat& other) const {
        Mat result;
        for (int i = 0; i < R * C; ++i) result.data[i] = data[i] - other.data[i];
        return result;
    }

    Mat operator-() const {
        Mat result;
        for (int i = 0; i < R * C; ++i) result.data[i] = -data[i];
        return result;
    }

    Mat operator*(double s) const {
        Mat result;
        for (int i = 0; i < R * C; ++i) result.data[i] = data[i] * s;
        return result;
    }

    Mat operator/(double s) const {
        Mat result;
        for (int i = 0; i < R * C; ++i) result.data[i] = data[i] / s;
        return result;
    }

    Mat& operator+=(const Mat& other) {
        for (int i = 0; i < R * C; ++i) data[i] += other.data[i];
        return *this;
    }

    Mat& operator-=(const Mat& other) {
        for (int i = 0; i < R * C; ++i) data[i] -= other.data[i];
        return *this;
    }

    Mat& operator*=(double s) {
        for (int i = 0; i < R * C; ++i) data[i] *= s;
        return *this;
    }

    // Matrix-Matrix multiplication
    template<int C2>
    Mat<R, C2> operator*(const Mat<C, C2>& other) const {
        Mat<R, C2> result;
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C2; ++j) {
                double sum = 0;
                for (int k = 0; k < C; ++k) {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }

    // Matrix-Vector multiplication
    Vec<R> operator*(const Vec<C>& v) const {
        Vec<R> result;
        for (int i = 0; i < R; ++i) {
            double sum = 0;
            for (int j = 0; j < C; ++j) {
                sum += (*this)(i, j) * v(j);
            }
            result(i) = sum;
        }
        return result;
    }

    // Transpose
    Mat<C, R> transpose() const {
        Mat<C, R> result;
        for (int i = 0; i < R; ++i) {
            for (int j = 0; j < C; ++j) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }

    // Trace (sum of diagonal elements)
    double trace() const {
        static_assert(R == C, "Trace only for square matrices");
        double sum = 0;
        for (int i = 0; i < R; ++i) sum += (*this)(i, i);
        return sum;
    }

    // Get column as vector
    Vec<R> col(int c) const {
        Vec<R> result;
        for (int i = 0; i < R; ++i) result(i) = (*this)(i, c);
        return result;
    }

    // Set column from vector
    void setCol(int c, const Vec<R>& v) {
        for (int i = 0; i < R; ++i) (*this)(i, c) = v(i);
    }

    // Get row as vector
    Vec<C> row(int r) const {
        Vec<C> result;
        for (int j = 0; j < C; ++j) result(j) = (*this)(r, j);
        return result;
    }

    // Set row from vector
    void setRow(int r, const Vec<C>& v) {
        for (int j = 0; j < C; ++j) (*this)(r, j) = v(j);
    }

    // Get diagonal as vector (for square matrices)
    Vec<R> diagonal() const {
        static_assert(R == C, "Diagonal only for square matrices");
        Vec<R> result;
        for (int i = 0; i < R; ++i) result(i) = (*this)(i, i);
        return result;
    }

    // Set diagonal from vector
    void setDiagonal(const Vec<R>& v) {
        static_assert(R == C, "setDiagonal only for square matrices");
        for (int i = 0; i < R; ++i) (*this)(i, i) = v(i);
    }

    // Normalize each column
    void normalizeColumns() {
        for (int c = 0; c < C; ++c) {
            Vec<R> v = col(c);
            v.normalize();
            setCol(c, v);
        }
    }

    // Add vector to each column (colwise += )
    void colwiseAdd(const Vec<R>& v) {
        for (int c = 0; c < C; ++c) {
            for (int r = 0; r < R; ++r) {
                (*this)(r, c) += v(r);
            }
        }
    }
};

// Scalar * Matrix
template<int R, int C>
Mat<R, C> operator*(double s, const Mat<R, C>& m) { return m * s; }

// ============================================================================
// SPECIAL MATRIX OPERATIONS
// ============================================================================

// 3x3 Matrix Inverse (Explicit formula using cofactors)
inline Mat<3, 3> inverse3x3(const Mat<3, 3>& m) {
    Mat<3, 3> result;
    
    double a = m(0,0), b = m(0,1), c = m(0,2);
    double d = m(1,0), e = m(1,1), f = m(1,2);
    double g = m(2,0), h = m(2,1), i = m(2,2);
    
    double det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
#ifdef DEBUG
    #include <cassert>
    if (std::abs(det) < 1e-15) {
        assert(false && "Matrix is singular, cannot invert");
    }
#endif
    
    double invDet = 1.0 / det;
    
    result(0,0) = (e*i - f*h) * invDet;
    result(0,1) = (c*h - b*i) * invDet;
    result(0,2) = (b*f - c*e) * invDet;
    result(1,0) = (f*g - d*i) * invDet;
    result(1,1) = (a*i - c*g) * invDet;
    result(1,2) = (c*d - a*f) * invDet;
    result(2,0) = (d*h - e*g) * invDet;
    result(2,1) = (b*g - a*h) * invDet;
    result(2,2) = (a*e - b*d) * invDet;
    
    return result;
}

// 4x4 Matrix Inverse (Block inversion method)
inline Mat<4, 4> inverse4x4(const Mat<4, 4>& m) {
    // Use Gauss-Jordan elimination for 4x4
    Mat<4, 4> result = Mat<4, 4>::Identity();
    Mat<4, 4> aug = m;
    
    for (int col = 0; col < 4; ++col) {
        // Find pivot
        int pivot = col;
        double maxVal = std::abs(aug(col, col));
        for (int row = col + 1; row < 4; ++row) {
            if (std::abs(aug(row, col)) > maxVal) {
                maxVal = std::abs(aug(row, col));
                pivot = row;
            }
        }
        
        if (maxVal < 1e-15) {
#ifdef DEBUG
            assert(false && "Matrix is singular, cannot invert");
#endif
            // Singular matrix
            return Mat<4, 4>::Zero();
        }
        
        // Swap rows
        if (pivot != col) {
            for (int j = 0; j < 4; ++j) {
                std::swap(aug(col, j), aug(pivot, j));
                std::swap(result(col, j), result(pivot, j));
            }
        }
        
        // Scale pivot row
        double scale = 1.0 / aug(col, col);
        for (int j = 0; j < 4; ++j) {
            aug(col, j) *= scale;
            result(col, j) *= scale;
        }
        
        // Eliminate column
        for (int row = 0; row < 4; ++row) {
            if (row != col) {
                double factor = aug(row, col);
                for (int j = 0; j < 4; ++j) {
                    aug(row, j) -= factor * aug(col, j);
                    result(row, j) -= factor * result(col, j);
                }
            }
        }
    }
    
    return result;
}

// 6x6 Matrix Inverse (Gauss-Jordan elimination)
inline Mat<6, 6> inverse6x6(const Mat<6, 6>& m) {
    Mat<6, 6> result = Mat<6, 6>::Identity();
    Mat<6, 6> aug = m;
    
    for (int col = 0; col < 6; ++col) {
        // Find pivot
        int pivot = col;
        double maxVal = std::abs(aug(col, col));
        for (int row = col + 1; row < 6; ++row) {
            if (std::abs(aug(row, col)) > maxVal) {
                maxVal = std::abs(aug(row, col));
                pivot = row;
            }
        }
        
        if (maxVal < 1e-15) {
#ifdef DEBUG
            assert(false && "Matrix is singular, cannot invert");
#endif
            return Mat<6, 6>::Zero();
        }
        
        // Swap rows
        if (pivot != col) {
            for (int j = 0; j < 6; ++j) {
                std::swap(aug(col, j), aug(pivot, j));
                std::swap(result(col, j), result(pivot, j));
            }
        }
        
        // Scale pivot row
        double scale = 1.0 / aug(col, col);
        for (int j = 0; j < 6; ++j) {
            aug(col, j) *= scale;
            result(col, j) *= scale;
        }
        
        // Eliminate column
        for (int row = 0; row < 6; ++row) {
            if (row != col) {
                double factor = aug(row, col);
                for (int j = 0; j < 6; ++j) {
                    aug(row, j) -= factor * aug(col, j);
                    result(row, j) -= factor * result(col, j);
                }
            }
        }
    }
    
    return result;
}

// Pseudoinverse for 3x4 matrix: S+ = S^T * (S * S^T)^(-1)
inline Mat<4, 3> pseudoInverse3x4(const Mat<3, 4>& S) {
    Mat<4, 3> St = S.transpose();
    Mat<3, 3> SSt = S * St;
    Mat<3, 3> SSt_inv = inverse3x3(SSt);
    return St * SSt_inv;
}

// Outer product: v1 * v2^T
template<int R, int C>
Mat<R, C> outerProduct(const Vec<R>& v1, const Vec<C>& v2) {
    Mat<R, C> result;
    for (int i = 0; i < R; ++i) {
        for (int j = 0; j < C; ++j) {
            result(i, j) = v1(i) * v2(j);
        }
    }
    return result;
}

// ============================================================================
// EIGENSOLVER FOR 4x4 SYMMETRIC MATRICES (Jacobi Eigenvalue Algorithm)
// Used by QUEST algorithm - guaranteed convergence for symmetric matrices
// ============================================================================
struct EigenResult4 {
    double eigenvalue;
    Vec<4> eigenvector;
    
    // Implicit conversion to Vec<4> (returns eigenvector)
    operator Vec<4>() const { return eigenvector; }
};

// Full eigendecomposition result (all eigenvalues and eigenvectors)
struct EigenDecomp4 {
    Vec<4> eigenvalues;      // Sorted descending (largest first)
    Mat<4, 4> eigenvectors;  // Columns are eigenvectors (column i corresponds to eigenvalue i)
};

// Jacobi eigenvalue algorithm for 4x4 symmetric matrices
// Guaranteed convergent for symmetric matrices
// Returns eigenvalues sorted descending, with corresponding eigenvectors as columns
inline EigenDecomp4 jacobiEigen4x4(const Mat<4, 4>& A, int maxSweeps = 50, double tol = 1e-12) {
    Mat<4, 4> V = Mat<4, 4>::Identity();  // Eigenvector accumulator
    Mat<4, 4> D = A;                       // Working copy (will become diagonal)
    
    for (int sweep = 0; sweep < maxSweeps; ++sweep) {
        // Find largest off-diagonal element
        double maxOffDiag = 0.0;
        int p = 0, q = 1;
        
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                double absVal = std::abs(D(i, j));
                if (absVal > maxOffDiag) {
                    maxOffDiag = absVal;
                    p = i;
                    q = j;
                }
            }
        }
        
        // Check convergence (matrix is diagonal enough)
        if (maxOffDiag < tol) break;
        
        // Compute Jacobi rotation angle
        double app = D(p, p);
        double aqq = D(q, q);
        double apq = D(p, q);
        
        double theta;
        if (std::abs(aqq - app) < 1e-15) {
            // Avoid division by zero when diagonal elements are equal
            theta = (apq > 0) ? PI / 4.0 : -PI / 4.0;
        } else {
            theta = 0.5 * std::atan2(2.0 * apq, aqq - app);
        }
        
        double c = std::cos(theta);
        double s = std::sin(theta);
        
        // Apply rotation to D: D = J^T * D * J
        // Update rows p and q
        for (int j = 0; j < 4; ++j) {
            double dp = D(p, j);
            double dq = D(q, j);
            D(p, j) = c * dp - s * dq;
            D(q, j) = s * dp + c * dq;
        }
        // Update columns p and q
        for (int i = 0; i < 4; ++i) {
            double dp = D(i, p);
            double dq = D(i, q);
            D(i, p) = c * dp - s * dq;
            D(i, q) = s * dp + c * dq;
        }
        // The (p,q) and (q,p) elements are now zero (within tolerance)
        D(p, q) = 0.0;
        D(q, p) = 0.0;
        
        // Accumulate eigenvectors: V = V * J
        for (int i = 0; i < 4; ++i) {
            double vp = V(i, p);
            double vq = V(i, q);
            V(i, p) = c * vp - s * vq;
            V(i, q) = s * vp + c * vq;
        }
    }
    
    // Extract eigenvalues from diagonal
    EigenDecomp4 result;
    for (int i = 0; i < 4; ++i) {
        result.eigenvalues(i) = D(i, i);
    }
    result.eigenvectors = V;
    
    // Sort by descending eigenvalue (so maximum is first)
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            if (result.eigenvalues(j) > result.eigenvalues(i)) {
                // Swap eigenvalues
                double temp = result.eigenvalues(i);
                result.eigenvalues(i) = result.eigenvalues(j);
                result.eigenvalues(j) = temp;
                // Swap eigenvector columns
                for (int k = 0; k < 4; ++k) {
                    double tempV = result.eigenvectors(k, i);
                    result.eigenvectors(k, i) = result.eigenvectors(k, j);
                    result.eigenvectors(k, j) = tempV;
                }
            }
        }
    }
    
    return result;
}

// Convenience function: Get just the maximum eigenvector (for QUEST compatibility)
// Maintains same interface as before for drop-in replacement
inline EigenResult4 maxEigenvector4x4Symmetric(const Mat<4, 4>& A) {
    EigenDecomp4 decomp = jacobiEigen4x4(A);
    EigenResult4 result{
        decomp.eigenvalues(0),        // Largest eigenvalue
        decomp.eigenvectors.col(0)    // Corresponding eigenvector
    };
    // NaN safety: if eigenvector has NaN, return identity quaternion
    if (result.eigenvector.hasNaN()) {
        result.eigenvector = Vec<4>{1.0, 0.0, 0.0, 0.0};
        result.eigenvalue = 0.0;
    }
    return result;
}


// ============================================================================
// SKEW-SYMMETRIC MATRIX (for cross product as matrix)
// ============================================================================
inline Mat<3, 3> skew(const Vec<3>& v) {
    Mat<3, 3> S;
    S(0, 0) = 0.0;    S(0, 1) = -v(2);  S(0, 2) = v(1);
    S(1, 0) = v(2);   S(1, 1) = 0.0;    S(1, 2) = -v(0);
    S(2, 0) = -v(1);  S(2, 1) = v(0);   S(2, 2) = 0.0;
    return S;
}

// ============================================================================
// CONVENIENCE TYPE ALIASES
// ============================================================================
using Vec3 = Vec<3>;
using Vec4 = Vec<4>;
using Vec6 = Vec<6>;
using Vec7 = Vec<7>;
using Vec10 = Vec<10>;
using Vec17 = Vec<17>;
using Vec29 = Vec<29>;

using Mat2 = Mat<2, 2>;
using Mat3 = Mat<3, 3>;
using Mat4 = Mat<4, 4>;
using Mat6 = Mat<6, 6>;
using Mat34 = Mat<3, 4>;
using Mat43 = Mat<4, 3>;
using Mat36 = Mat<3, 6>;
using Mat63 = Mat<6, 3>;

} // namespace Math

// ============================================================================
// STREAM OUTPUT (for debugging)
// ============================================================================
#ifdef CORE_MATH_ENABLE_IOSTREAM
#include <iostream>

template<int N>
std::ostream& operator<<(std::ostream& os, const Math::Vec<N>& v) {
    os << "[";
    for (int i = 0; i < N; ++i) {
        os << v(i);
        if (i < N - 1) os << ", ";
    }
    os << "]";
    return os;
}

template<int R, int C>
std::ostream& operator<<(std::ostream& os, const Math::Mat<R, C>& m) {
    os << "[\n";
    for (int i = 0; i < R; ++i) {
        os << "  [";
        for (int j = 0; j < C; ++j) {
            os << m(i, j);
            if (j < C - 1) os << ", ";
        }
        os << "]\n";
    }
    os << "]";
    return os;
}
#endif // CORE_MATH_ENABLE_IOSTREAM

#endif // CORE_MATH_HPP
