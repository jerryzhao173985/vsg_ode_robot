#ifndef AXIS_H
#define AXIS_H

#include <vsg/maths/vec3.h>
#include <vsg/maths/vec4.h>
#include <ode/ode.h>
#include <iostream>

namespace lpzrobots {

// Class for axis. This is internally a 4 dimensional vector (homogeneous) with last component = 0
// meaning it is a direction not a point
class Axis : public vsg::vec4 {
public:
    // Axis() : vsg::vec4() {}
    // Axis(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    // Axis(float x, float y, float z) : vsg::vec4(x, y, z, 0.0f) {}
    Axis() : vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f) {}
    Axis(double x_, double y_, double z_) 
        : vsg::vec4(static_cast<float>(x_), 
                    static_cast<float>(y_), 
                    static_cast<float>(z_), 0.0f) {}
    Axis(const vsg::vec4& v) : vsg::vec4(v) { this->w = 0.0f; }
    Axis(const vsg::dvec3& v) : vsg::vec4(v.x, v.y, v.z, 0.0f) {}
    Axis(const dReal v[3]) : vsg::vec4(v[0], v[1], v[2], 0.0f) {}

    // Convert to VSG vector
    vsg::dvec3 toVec3() const {
        return vsg::dvec3(x, y, z);
    }

    vsg::dvec3 vec3() const { 
        return vsg::dvec3(x, y, z); 
    }

    float enclosingAngle(const Axis& a) const {
        return acos(dot(a) / (length() * a.length()));
    }

    Axis crossProduct(const Axis& a) const {
        return Axis(y * a.z - z * a.y,
                   z * a.x - x * a.z,
                   x * a.y - y * a.x);
    }

    void print() {
        std::cout << '(' << x << ',' << y << ',' << z << ')' << std::endl;
    }

    // VSG specific helper methods for vector operations
    float dot(const Axis& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    float length() const {
        return sqrt(dot(*this));
    }
    
private:
    double x, y, z;
};

}
#endif