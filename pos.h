#ifndef __POS_H
#define __POS_H

#include <iostream>
#include <vsg/maths/vec3.h>
#include <vsg/maths/vec4.h>
#include <ode/ode.h>
#include "position.h"

namespace lpzrobots {

class Pos : public vsg::dvec3 {
public:
    // Constructors
    Pos() : vsg::dvec3() {}
    Pos(double x, double y, double z) : vsg::dvec3(x, y, z) {}
    Pos(const vsg::dvec3& v) : vsg::dvec3(v) {}
    Pos(const vsg::dvec4& v) : vsg::dvec3(v.x, v.y, v.z) {}
    Pos (const Position& p)  : vsg::dvec3(p.x, p.y, p.z) {}
    Pos(const dReal v[3])    : vsg::dvec3(v[0], v[1], v[2]) {}

    // Access methods to maintain compatibility - using proper member access
    double x() const { return this->operator[](0); }
    double y() const { return this->operator[](1); }
    double z() const { return this->operator[](2); }

    // Operators
    /// scaling
    Pos operator*(double f) const { return Pos(x()*f,y()*f,z()*f);}
    /// scalar product
    double operator*(const Pos& p) const { return p.x()*x() + p.y()*y() + p.z()*z();}
    /// componentwise  product
    Pos operator&(const Pos& p) const { return Pos(p.x()*x(), p.y()*y(), p.z()*z());}

    Position toPosition(){
      return Position(x(), y(), z());
    }

    void print() const {
      std::cout << '(' << x() << ',' << y() << ',' << z() << ')' << std::endl;
    }
};

} // namespace lpzrobots

#endif // __POS_H