#ifndef AXIS_H
#define AXIS_H

#include <vsg/maths/vec3.h>
#include <vsg/maths/vec4.h>
#include <vsg/maths/mat4.h>
#include <ode/ode.h>
#include <iostream>
#include <cmath>

namespace lpzrobots {

/**
 * Represents a directional axis in 3D space using homogeneous coordinates.
 * This is internally a 4D vector with w=0, representing a direction rather than a point.
 * Compatible with VSG transformation operations while maintaining direction vector semantics.
 */
class Axis {
public:
    // Standard constructors
    Axis() : _vec(0.0, 0.0, 0.0, 0.0) {}
    
    Axis(double x, double y, double z) : _vec(x, y, z, 0.0) {
        // We don't normalize by default to match original behavior
        // Use normalizedAxis() if unit vector is needed
    }

    // Copy constructor and assignment
    Axis(const Axis& other) = default;
    Axis& operator=(const Axis& other) = default;

    // Conversion constructors - explicit to prevent accidental conversions
    explicit Axis(const vsg::dvec4& v) : _vec(v.x, v.y, v.z, 0.0) {}
    explicit Axis(const vsg::dvec3& v) : _vec(v.x, v.y, v.z, 0.0) {}
    explicit Axis(const dReal v[3]) : _vec(v[0], v[1], v[2], 0.0) {}

    // Standard axis factories
    static Axis X() { return Axis(1.0, 0.0, 0.0); }
    static Axis Y() { return Axis(0.0, 1.0, 0.0); }
    static Axis Z() { return Axis(0.0, 0.0, 1.0); }

    // Accessors
    double x() const { return _vec.x; }
    double y() const { return _vec.y; }
    double z() const { return _vec.z; }
    double w() const { return _vec.w; } // Always 0 for direction vectors

    // Conversion methods
    vsg::dvec3 vec3() const { return vsg::dvec3(_vec.x, _vec.y, _vec.z); }
    vsg::dvec3 toVec3() const { return vsg::dvec3(_vec.x, _vec.y, _vec.z); } // old  
    const vsg::dvec4& vec4() const { return _vec; }

    // Core geometric operations
    double dot(const Axis& rhs) const {
        return _vec.x * rhs._vec.x + _vec.y * rhs._vec.y + _vec.z * rhs._vec.z;
    }

    double length() const {
        return std::sqrt(dot(*this));
    }

    Axis normalize() const {
        double len = length();
        if (len > 0.0) {
            return Axis(_vec.x / len, _vec.y / len, _vec.z / len);
        }
        return *this;
    }

    double enclosingAngle(const Axis& other) const {
        // Compute angle between axes using normalized vectors for numerical stability
        Axis a1 = this->normalize();
        Axis a2 = other.normalize();
        double cosAngle = a1.dot(a2);
        // Clamp to [-1,1] to handle numerical imprecision
        cosAngle = std::max(-1.0, std::min(1.0, cosAngle));
        return std::acos(cosAngle);
    }

    Axis crossProduct(const Axis& rhs) const {
        return Axis(
            _vec.y * rhs._vec.z - _vec.z * rhs._vec.y,
            _vec.z * rhs._vec.x - _vec.x * rhs._vec.z,
            _vec.x * rhs._vec.y - _vec.y * rhs._vec.x
        );
    }

    // Transform operations
    Axis transform(const vsg::dmat4& matrix) const {
        vsg::dvec4 result = matrix * _vec;
        // Note: w component is forced to 0 as this is a direction vector
        return Axis(result.x, result.y, result.z);
    }

    // Operators for VSG compatibility
    operator vsg::dvec4() const { return _vec; }
    operator vsg::dvec3() const { return vec3(); }

    // Matrix multiplication operators that match VSG semantics
    Axis operator*(const vsg::dmat4& matrix) const {
        // Row vector multiplication (vec * mat)
        vsg::dvec4 result = _vec * matrix;
        return Axis(result.x, result.y, result.z);
    }

    friend Axis operator*(const vsg::dmat4& matrix, const Axis& axis) {
        // Column vector multiplication (mat * vec)
        vsg::dvec4 result = matrix * axis._vec;
        return Axis(result.x, result.y, result.z);
    }

    // Comparison operators
    bool operator==(const Axis& rhs) const {
        const double epsilon = 1e-10;
        return std::abs(_vec.x - rhs._vec.x) < epsilon &&
               std::abs(_vec.y - rhs._vec.y) < epsilon &&
               std::abs(_vec.z - rhs._vec.z) < epsilon;
    }

    bool operator!=(const Axis& rhs) const {
        return !(*this == rhs);
    }

    // Debug output
    void print() const {
        std::cout << "Axis(" << _vec.x << ", " << _vec.y << ", " 
                 << _vec.z << ")" << std::endl;
    }

    // Array access operator for compatibility with C-style array usage
    double operator[](size_t i) const {
        switch(i) {
            case 0: return _vec.x;
            case 1: return _vec.y;
            case 2: return _vec.z;
            default: throw std::out_of_range("Axis index out of range");
        }
    }

    // Helper methods
    bool isZero(double epsilon = 1e-10) const {
        return length() < epsilon;
    }

private:
    vsg::dvec4 _vec;  // Homogeneous coordinates (w=0 for direction vector)
};

// Utility functions for common transformations
inline Axis rotateAxis(const Axis& axis, const vsg::dmat4& rotation) {
    return axis.transform(rotation);
}

inline Axis normalizedAxis(const Axis& axis) {
    return axis.normalize();
}

} // namespace lpzrobots

#endif // AXIS_H