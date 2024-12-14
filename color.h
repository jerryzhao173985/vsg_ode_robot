// color.h - VSG Version

#ifndef __COLOR_H
#define __COLOR_H

#include <vsg/maths/vec4.h>
#include <iostream>

namespace lpzrobots {

class Color : public vsg::vec4
{
public:
    // Default constructor initializes to white color
    Color() : vsg::vec4(1.0f, 1.0f, 1.0f, 1.0f) {}
    // Copy constructor from vsg::vec4
    Color(const vsg::vec4& color)
        : vsg::vec4(color) {}
    // Constructor with RGB values, alpha defaults to 1.0
    Color(float r, float g, float b)
        : vsg::vec4(r, g, b, 1.0f) {}
    // Constructor with RGBA values
    Color(float r, float g, float b, float a)
        : vsg::vec4(r, g, b, a) {}

    // Static method to create Color from 0-255 RGB(A) values
    static Color rgb255(unsigned char r, unsigned char g, unsigned char b,
                        unsigned char a = 255) {
        return Color(static_cast<float>(r) / 255.0f,
                     static_cast<float>(g) / 255.0f,
                     static_cast<float>(b) / 255.0f,
                     static_cast<float>(a) / 255.0f);
    }

    // Method to print the color components
    void print(std::ostream& out) const {
        out << '(' << r() << ',' << g() << ',' << b() << ',' << a() << ')';
    }

    // Overloaded operator<< for easy printing
    friend std::ostream& operator<<(std::ostream& out, const Color& col) {
        col.print(out);
        return out;
    }

    // Accessors and mutators for color components
    float r() const { return x; }
    float& r() { return x; }
    float g() const { return y; }
    float& g() { return y; }
    float b() const { return z; }
    float& b() { return z; }
    float a() const { return w; }
    float& a() { return w; }

    // Alias for alpha component
    float alpha() const { return w; }
    float& alpha() { return w; }

    // Undef color constant
    static const Color undef;
};

// Definition of the undef color constant (e.g., fully transparent black)
// const Color Color::undef = Color(0.0f, 0.0f, 0.0f, 0.0f);

} // namespace lpzrobots

#endif // __COLOR_H
