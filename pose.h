#ifndef __POSE_H
#define __POSE_H

#include <vsg/maths/mat4.h>
#include <vsg/maths/quat.h>
#include "pos.h"

namespace lpzrobots {

// Add matrix inverse function if not provided by VSG
inline vsg::dmat4 inverse_matrix(const vsg::dmat4& m) {
    vsg::dmat4 inv;
    double det;

    // First row of cofactors
    inv[0][0] = m[1][1] * m[2][2] * m[3][3] + m[1][2] * m[2][3] * m[3][1] + m[1][3] * m[2][1] * m[3][2] -
                m[1][1] * m[2][3] * m[3][2] - m[1][2] * m[2][1] * m[3][3] - m[1][3] * m[2][2] * m[3][1];

    inv[0][1] = m[0][1] * m[2][3] * m[3][2] + m[0][2] * m[2][1] * m[3][3] + m[0][3] * m[2][2] * m[3][1] -
                m[0][1] * m[2][2] * m[3][3] - m[0][2] * m[2][3] * m[3][1] - m[0][3] * m[2][1] * m[3][2];

    inv[0][2] = m[0][1] * m[1][2] * m[3][3] + m[0][2] * m[1][3] * m[3][1] + m[0][3] * m[1][1] * m[3][2] -
                m[0][1] * m[1][3] * m[3][2] - m[0][2] * m[1][1] * m[3][3] - m[0][3] * m[1][2] * m[3][1];

    inv[0][3] = m[0][1] * m[1][3] * m[2][2] + m[0][2] * m[1][1] * m[2][3] + m[0][3] * m[1][2] * m[2][1] -
                m[0][1] * m[1][2] * m[2][3] - m[0][2] * m[1][3] * m[2][1] - m[0][3] * m[1][1] * m[2][2];

    // Second row of cofactors
    inv[1][0] = m[1][0] * m[2][3] * m[3][2] + m[1][2] * m[2][0] * m[3][3] + m[1][3] * m[2][2] * m[3][0] -
                m[1][0] * m[2][2] * m[3][3] - m[1][2] * m[2][3] * m[3][0] - m[1][3] * m[2][0] * m[3][2];

    inv[1][1] = m[0][0] * m[2][2] * m[3][3] + m[0][2] * m[2][3] * m[3][0] + m[0][3] * m[2][0] * m[3][2] -
                m[0][0] * m[2][3] * m[3][2] - m[0][2] * m[2][0] * m[3][3] - m[0][3] * m[2][2] * m[3][0];

    inv[1][2] = m[0][0] * m[1][3] * m[3][2] + m[0][2] * m[1][0] * m[3][3] + m[0][3] * m[1][2] * m[3][0] -
                m[0][0] * m[1][2] * m[3][3] - m[0][2] * m[1][3] * m[3][0] - m[0][3] * m[1][0] * m[3][2];

    inv[1][3] = m[0][0] * m[1][2] * m[2][3] + m[0][2] * m[1][3] * m[2][0] + m[0][3] * m[1][0] * m[2][2] -
                m[0][0] * m[1][3] * m[2][2] - m[0][2] * m[1][0] * m[2][3] - m[0][3] * m[1][2] * m[2][0];

    // Third row of cofactors
    inv[2][0] = m[1][0] * m[2][1] * m[3][3] + m[1][1] * m[2][3] * m[3][0] + m[1][3] * m[2][0] * m[3][1] -
                m[1][0] * m[2][3] * m[3][1] - m[1][1] * m[2][0] * m[3][3] - m[1][3] * m[2][1] * m[3][0];

    inv[2][1] = m[0][0] * m[2][3] * m[3][1] + m[0][1] * m[2][0] * m[3][3] + m[0][3] * m[2][1] * m[3][0] -
                m[0][0] * m[2][1] * m[3][3] - m[0][1] * m[2][3] * m[3][0] - m[0][3] * m[2][0] * m[3][1];

    inv[2][2] = m[0][0] * m[1][1] * m[3][3] + m[0][1] * m[1][3] * m[3][0] + m[0][3] * m[1][0] * m[3][1] -
                m[0][0] * m[1][3] * m[3][1] - m[0][1] * m[1][0] * m[3][3] - m[0][3] * m[1][1] * m[3][0];

    inv[2][3] = m[0][0] * m[1][3] * m[2][1] + m[0][1] * m[1][0] * m[2][3] + m[0][3] * m[1][1] * m[2][0] -
                m[0][0] * m[1][1] * m[2][3] - m[0][1] * m[1][3] * m[2][0] - m[0][3] * m[1][0] * m[2][1];

    // Fourth row of cofactors
    inv[3][0] = m[1][0] * m[2][2] * m[3][1] + m[1][1] * m[2][0] * m[3][2] + m[1][2] * m[2][1] * m[3][0] -
                m[1][0] * m[2][1] * m[3][2] - m[1][1] * m[2][2] * m[3][0] - m[1][2] * m[2][0] * m[3][1];

    inv[3][1] = m[0][0] * m[2][1] * m[3][2] + m[0][1] * m[2][2] * m[3][0] + m[0][2] * m[2][0] * m[3][1] -
                m[0][0] * m[2][2] * m[3][1] - m[0][1] * m[2][0] * m[3][2] - m[0][2] * m[2][1] * m[3][0];

    inv[3][2] = m[0][0] * m[1][2] * m[3][1] + m[0][1] * m[1][0] * m[3][2] + m[0][2] * m[1][1] * m[3][0] -
                m[0][0] * m[1][1] * m[3][2] - m[0][1] * m[1][2] * m[3][0] - m[0][2] * m[1][0] * m[3][1];

    inv[3][3] = m[0][0] * m[1][1] * m[2][2] + m[0][1] * m[1][2] * m[2][0] + m[0][2] * m[1][0] * m[2][1] -
                m[0][0] * m[1][2] * m[2][1] - m[0][1] * m[1][0] * m[2][2] - m[0][2] * m[1][1] * m[2][0];

    // Calculate determinant using first row
    det = m[0][0] * inv[0][0] + m[0][1] * inv[1][0] + m[0][2] * inv[2][0] + m[0][3] * inv[3][0];

    if (det == 0)
        return vsg::dmat4(1.0); // Return identity if not invertible

    det = 1.0 / det;

    // Multiply all elements by inverse determinant
    for (int i = 0; i < 16; i++)
        inv.data()[i] *= det;

    return inv;
}

class Pose : public vsg::dmat4 {
public:
    // Constructors
    Pose() : vsg::dmat4(1.0) {}
    Pose(const vsg::dmat4& m) : vsg::dmat4(m) {}

    // Direct matrix access
    const vsg::dmat4& matrix() const { return *this; }
    vsg::dmat4& matrix() { return *this; }

    const vsg::dmat4& getMatrix() const { return *this; }
    vsg::dmat4& getMatrix() { return *this; }

    // Constructor from ODE position and rotation
    Pose(const double* position, const double* rotation) 
        : vsg::dmat4(rotation[0], rotation[1], rotation[2], 0.0,
                     rotation[4], rotation[5], rotation[6], 0.0,
                     rotation[8], rotation[9], rotation[10], 0.0,
                     position[0], position[1], position[2], 1.0) {}

    // Matrix operations
    Pose inverse() const {
        return Pose(inverse_matrix(*this));
    }

    // // Matrix operations
    // Pose inverse() const {
    //     return Pose(vsg::inverse(static_cast<const vsg::dmat4&>(*this)));
    // }

    // Get transformation components
    Pos getTrans() const {
        return Pos((*this)[3][0], (*this)[3][1], (*this)[3][2]);
    }

    // set transformation components
    void setTrans(const vsg::dvec3& pos) {
        (*this)[3][0] = pos[0];
        (*this)[3][1] = pos[1];
        (*this)[3][2] = pos[2];
    }

    vsg::dquat getRotate() const {
        const double m00 = (*this)[0][0], m01 = (*this)[0][1], m02 = (*this)[0][2];
        const double m10 = (*this)[1][0], m11 = (*this)[1][1], m12 = (*this)[1][2];
        const double m20 = (*this)[2][0], m21 = (*this)[2][1], m22 = (*this)[2][2];

        double w, x, y, z;
        const double trace = m00 + m11 + m22;

        if (trace > 0.0) {
            const double s = 0.5 / std::sqrt(trace + 1.0);
            w = 0.25 / s;
            x = (m21 - m12) * s;
            y = (m02 - m20) * s;
            z = (m10 - m01) * s;
        } else if (m00 > m11 && m00 > m22) {
            const double s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
            w = (m21 - m12) / s;
            x = 0.25 * s;
            y = (m01 + m10) / s;
            z = (m02 + m20) / s;
        } else if (m11 > m22) {
            const double s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
            w = (m02 - m20) / s;
            x = (m01 + m10) / s;
            y = 0.25 * s;
            z = (m12 + m21) / s;
        } else {
            const double s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
            w = (m10 - m01) / s;
            x = (m02 + m20) / s;
            y = (m12 + m21) / s;
            z = 0.25 * s;
        }

        return vsg::dquat(x, y, z, w);
    }
};

} // namespace lpzrobots

#endif