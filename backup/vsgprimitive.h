// vsgprimitive.h - VSG Version

#ifndef __VSGPRIMITIVE_H
#define __VSGPRIMITIVE_H

#include <string>
#include <vector>
#include <vsg/all.h>
#include "vsghandle.h"
#include "color.h"
#include "pose.h"

namespace lpzrobots {

    /**
       Holds texture file and repeat information.
    */
    class TextureDescr {
    public:
        TextureDescr() : repeatOnR(1.0), repeatOnS(1.0) {}
        /**
           If repeatOnR or repeatOnS is negative, then it is used as a unit length for the texture.
        */
        TextureDescr(const std::string& filename, double repeatOnR = 1.0, double repeatOnS = 1.0)
            : filename(filename), repeatOnR(repeatOnR), repeatOnS(repeatOnS)
        {
        }
        std::string filename;
        double repeatOnR;
        double repeatOnS;
    };

    /**
       Interface class for graphical primitives like spheres, boxes, and meshes,
       which can be drawn by VSG. The idea is to hide all the details of the VSG
       implementation.
    */
    class VSGPrimitive : public vsg::Inherit<vsg::Object, VSGPrimitive> {
    public:
        enum Quality { Low, Middle, High };

        VSGPrimitive();
        virtual ~VSGPrimitive();

        /** Initialization of the primitive. Must be called to place the object into the scene.
            This function should be overloaded.
        */
        virtual void init(const VsgHandle& vsgHandle, Quality quality = Middle) = 0;

        /// Sets the transformation matrix of this object (position and orientation)
        virtual void setMatrix(const vsg::dmat4& m4x4);

        /// Returns the group node which is the root of all subcomponents of this primitive
        virtual vsg::ref_ptr<vsg::Group> getGroup();

        /// Assigns a texture to the primitive
        virtual void setTexture(const std::string& filename);
        /// Assigns a texture to the primitive, you can choose how often to repeat
        virtual void setTexture(const TextureDescr& texture);
        /// Assigns a texture to the x-th surface of the primitive, you can choose how often to repeat
        virtual void setTexture(int surface, const TextureDescr& texture);
        /// Assign a set of textures to the surfaces of the primitive
        virtual void setTextures(const std::vector<TextureDescr>& textures);
        /// Returns the list of textures
        virtual std::vector<TextureDescr> getTextures() const;

        /// Sets the color for painting this primitive
        virtual void setColor(const Color& color);
        /// Sets the color using the color schema of VsgHandle
        virtual void setColor(const std::string& color);
        /// Returns the current color
        virtual Color getColor() const;

        /// Returns a transformation node
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransform();

        /// Returns the VsgHandle object
        virtual const VsgHandle& getVsgHandle();

    protected:
        /// This actually sets the textures
        virtual void applyTextures();

        vsg::ref_ptr<vsg::MatrixTransform> transform;
        vsg::ref_ptr<vsg::Node> geometryNode;

        std::vector<TextureDescr> textures;

        VsgHandle vsgHandle;

        Color color;
    };

    // Derived classes

    /**
       A dummy graphical object, which has no representation in the graphical world.
    */
    class VSGDummy : public VSGPrimitive {
    public:
        VSGDummy();

        virtual void init(const VsgHandle& vsgHandle, Quality quality = Middle);
        virtual void setMatrix(const vsg::dmat4& m4x4);
        virtual vsg::ref_ptr<vsg::Group> getGroup();
        virtual void setTexture(const std::string& filename);
        virtual void setColor(const Color& color);
        /// Returns a transformation node
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransform();
    };

    /**
       Graphical plane (represented as a large thin box, because VSG does not draw infinite planes)
    */
    class VSGPlane : public VSGPrimitive {
    public:
        VSGPlane();

        virtual void init(const VsgHandle& vsgHandle, Quality quality = Middle);
    };

    /**
       Graphical box
    */
    class VSGBox : public VSGPrimitive {
    public:
        VSGBox(double lengthX, double lengthY, double lengthZ);
        VSGBox(const vsg::dvec3& dim);

        virtual void init(const VsgHandle& vsgHandle, Quality quality = Middle);

        virtual vsg::dvec3 getDim() const;
        virtual void setDim(const vsg::dvec3& dim);

    protected:
        vsg::dvec3 dim;
    };

    /**
       Graphical sphere
    */
    class VSGSphere : public VSGPrimitive {
    public:
        VSGSphere(double radius);

        virtual void init(const VsgHandle& vsgHandle, Quality quality = Middle);

        double getRadius() const { return radius; }

    protected:
        double radius;
    };

    /**
       Graphical capsule (a cylinder with round ends)
    */
    class VSGCapsule : public VSGPrimitive {
    public:
        VSGCapsule(double radius, double height);

        virtual void init(const VsgHandle& vsgHandle, Quality quality = Middle);

        double getRadius() const { return radius; }
        double getHeight() const { return height; }

    protected:
        double radius;
        double height;
    };

    /**
       Graphical cylinder
    */
    class VSGCylinder : public VSGPrimitive {
    public:
        VSGCylinder(double radius, double height);

        virtual void init(const VsgHandle& vsgHandle, Quality quality = Middle);

        double getRadius() const { return radius; }
        double getHeight() const { return height; }

    protected:
        double radius;
        double height;
    };

    /**
       Graphical line
    */
    class VSGLine : public VSGPrimitive {
    public:
        // The list of points is considered pairwise, start-end points of each line segment
        VSGLine(const std::list<vsg::dvec3>& points);

        virtual void init(const VsgHandle& vsgHandle, Quality quality = Middle);

        virtual void applyTextures() {}

        virtual void setColor(const Color& color);

        // Use the new points
        virtual void setPoints(const std::list<vsg::dvec3>& points);

    protected:
        std::list<vsg::dvec3> points;
        vsg::ref_ptr<vsg::Geometry> geometry;

        virtual void updatePoints();
    };

    /**
       Graphical Mesh or arbitrary VSG model.
    */
    class VSGMesh : public VSGPrimitive {
    public:
        /**
           Constructor
           @param filename Filename of the model file (search path is VSG data path)
           @param scale Scale factor used for scaling the model
        */
        VSGMesh(const std::string& filename, double scale = 1.0);

        virtual void init(const VsgHandle& vsgHandle, Quality quality = Middle);

        double getScale() const { return scale; }

    protected:
        std::string filename;
        double scale;

        vsg::ref_ptr<vsg::Node> meshNode;

        virtual void internInit(const VsgHandle& vsgHandle, bool loadAndDisplayMesh, Quality quality = Middle);
    };

    /**
       Text to be displayed on the HUD
    */
    class VSGText : public VSGPrimitive {
    public:
        VSGText(const std::string& text, int fontsize = 12 /* alignment parameter */);

        virtual ~VSGText();

        virtual void init(const VsgHandle& vsgHandle, Quality quality = Middle);
        virtual void setMatrix(const vsg::dmat4& m4x4);
        virtual vsg::ref_ptr<vsg::Group> getGroup();
        virtual void setColor(const Color& color);
        /// Returns a transformation node
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransform();

    private:
        // Text rendering components
        std::string text;
        int fontsize;
        // Alignment parameter
        // Need to define how to handle text alignment in VSG
    };

}

#endif // __VSGPRIMITIVE_H