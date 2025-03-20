// primitive.h - Revised VSG Version

#ifndef __PRIMITIVE_H
#define __PRIMITIVE_H

#include <ode/ode.h>
#include <vsg/all.h>

#include <vector>
#include "pos.h"
#include "pose.h"
#include "substance.h"
#include "color.h"
#include "odehandle.h"
#include "vsghandle.h"

namespace lpzrobots {

    // Helper functions
    Pose vsgPose(dGeomID geom);
    Pose vsgPose(dBodyID body);
    Pose vsgPose(const double* position, const double* rotation);
    void odeRotation(const Pose& pose, dMatrix3& odematrix);

    extern int globalNumVelocityViolations;

    /**
       Interface class for primitives represented in the physical and graphical world.
       This class integrates VSG and ODE, hiding most implementation details.
    */
    class Primitive : public vsg::Inherit<vsg::Object, Primitive> {
    public:
        /** Modes for initializing primitives */
        enum Modes {
            Body = 1,
            Geom = 2,
            Draw = 4,
            Density = 8,
            _Child = 16,
            _Transform = 32
        };
        enum Category { Dyn = 1, Stat = 2 };

        Primitive();
        virtual ~Primitive();

        /** Registers primitive in ODE and VSG.
            @param odeHandle Struct with ODE variables (world, space, etc.)
            @param mass Mass of the object in ODE (if withBody = true)
            @param VsgHandle Struct with VSG variables (scene node, color, etc.)
            @param mode Combination of Modes flags.
         */
        virtual void init(const OdeHandle& odeHandle, double mass,
                          const VsgHandle& vsgHandle,
                          char mode = Body | Geom | Draw) = 0;

        /** Updates the VSG nodes with ODE coordinates.
            Must be implemented by derived classes.
         */
        // virtual void update() = 0;
        virtual void update();

        /// Returns the associated VSG node, or nullptr if none
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode() = 0;

        /// Sets the color of the primitive
        virtual void setColor(const Color& color);

        void setMatrix(const vsg::dmat4& matrix);

        /// Sets the color using the color schema of VsgHandle
        // virtual void setColor(const std::string& color);

        /// Assigns a texture to the primitive
        // virtual void setTexture(const std::string& filename);
        // virtual void setTexture(const TextureDescr& texture);
        // virtual void setTexture(int surface, const TextureDescr& texture);
        // virtual void setTextures(const std::vector<TextureDescr>& textures);

        /// Sets the position of the primitive (orientation is preserved)
        virtual void setPosition(const Pos& pos);

        /// Sets the pose of the primitive
        virtual void setPose(const Pose& pose);

        /// Returns the position of the primitive
        virtual Pos getPosition() const;

        /// Returns the pose of the primitive
        virtual Pose getPose() const;

        /// Returns the linear velocity of the primitive
        virtual Pos getVel() const;

        /// Returns the angular velocity of the primitive
        virtual Pos getAngularVel() const;

        /** Applies a force to the primitive (in world coordinates)
            Returns true if successful.
        */
        virtual bool applyForce(vsg::dvec3 force);
        virtual bool applyForce(double x, double y, double z);

        /** Applies a torque to the primitive (in world coordinates)
            Returns true if successful.
        */
        virtual bool applyTorque(vsg::dvec3 torque);
        virtual bool applyTorque(double x, double y, double z);

        /** Sets the mass of the body (uniform)
            If density==true then mass is interpreted as a density
        */
        virtual void setMass(double mass, bool density = false) = 0;

        /** Sets full mass specification
            @param mass Total mass
            @param cgx, cgy, cgz Center of gravity coordinates
            @param I11, I22, I33, I12, I13, I23 Inertia tensor components
        */
        void setMass(double mass, double cgx, double cgy, double cgz,
                     double I11, double I22, double I33,
                     double I12, double I13, double I23);

        /// Returns the ODE geomID if available
        dGeomID getGeom() const { return geom; }

        /// Returns the ODE bodyID if available
        dBodyID getBody() const { return body; }

        /// Limits the linear velocity to maxVel
        bool limitLinearVel(double maxVel);

        /// Limits the angular velocity to maxVel
        bool limitAngularVel(double maxVel);

        /** Decelerates the primitive by applying a force and torque
            proportional to the velocities and given factors.
        */
        void decelerate(double factorLin, double factorAng);

        /// Transforms a point to local coordinates
        vsg::dvec3 toLocal(const vsg::dvec3& pos) const;

        /// Transforms a vector or axis to local coordinates
        vsg::dvec4 toLocal(const vsg::dvec4& axis) const;

        /// Transforms a point to global coordinates
        vsg::dvec3 toGlobal(const vsg::dvec3& pos) const;

        /// Transforms a vector or axis to global coordinates
        vsg::dvec4 toGlobal(const vsg::dvec4& axis) const;

        /** Sets whether to destroy the geom when the primitive is destroyed
            @param _destroyGeom If false, geom will not be destroyed
        */
        static void setDestroyGeomFlag(bool _destroyGeom) {
            destroyGeom = _destroyGeom;
        }

        /// Returns the number of velocity violations
        int getNumVelocityViolations() { return numVelocityViolations; }

        /// Sets the substance properties of the primitive
        void setSubstance(const Substance& substance);

        /* **** Storeable interface *******/
        virtual bool store(FILE* f) const;

        virtual bool restore(FILE* f);

    protected:
        /** Attaches geom to body (if any) and sets collision categories
            Assumes mode & Geom != 0
        */
        virtual void attachGeomAndSetColliderFlags();

    public:
        Substance substance; // Substance description

    protected:
        vsg::ref_ptr<vsg::Options> options;
        dGeomID geom;
        dBodyID body;
        char mode;
        bool substanceManuallySet;
        int numVelocityViolations; ///< Number of times the maximal velocity was exceeded

        static bool destroyGeom;

        // VSGPrimitive for graphical representation
        vsg::ref_ptr<vsg::Node> vsgPrimitive;
        vsg::ref_ptr<vsg::MatrixTransform> transformNode; // For convenience

        Color color;
    };

    /** Plane primitive */
    class Plane : public Primitive {
    public:
        Plane();
        virtual ~Plane();

        virtual void init(const OdeHandle& odeHandle, double mass,
                          const VsgHandle& vsgHandle,
                          char mode = Geom | Draw);

        virtual void update();
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode();

        virtual void setMass(double mass, bool density = false);

    };

    /** Box primitive */
    class Box : public Primitive {
    public:
        Box(double lengthX, double lengthY, double lengthZ);
        Box(const vsg::dvec3& dim);

        virtual ~Box();

        virtual void init(const OdeHandle& odeHandle, double mass,
                          const VsgHandle& vsgHandle,
                          char mode = Body | Geom | Draw);

        virtual void update();
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode();

        virtual void setMass(double mass, bool density = false);

        dReal getMass() const;
        dReal getVolume() const;     
        dReal getDensity() const; 

    protected:
        vsg::dvec3 dimensions;
    };

    /** Sphere primitive */
    class Sphere : public Primitive {
    public:
        Sphere(double radius);
        virtual ~Sphere();

        virtual void init(const OdeHandle& odeHandle, double mass,
                          const VsgHandle& vsgHandle,
                          char mode = Body | Geom | Draw);

        virtual void update();
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode();

        virtual void setMass(double mass, bool density = false);

    protected:
        float radius;
    };

    /** Additional primitives (Capsule, Cylinder, Ray, Mesh, Transform) would be defined similarly */

    class Capsule : public Primitive {
    public:
        Capsule(double radius, double height);
        virtual ~Capsule();
        
        virtual void init(const OdeHandle& odeHandle, double mass,
                        const VsgHandle& vsgHandle, char mode = Body | Geom | Draw);
        virtual void update();
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode();
        virtual void setMass(double mass, bool density = false);
        
    private:
        float radius;
        float height;
    };

    class Cone : public Primitive {
    public:
        Cone(double radius, double height);
        virtual ~Cone();
        
        virtual void init(const OdeHandle& odeHandle, double mass,
                        const VsgHandle& vsgHandle, char mode = Body | Geom | Draw);
        virtual void update();
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode();
        virtual void setMass(double mass, bool density = false);
        
    private:
        float radius;
        float height;
    };

    class Cylinder : public Primitive {
    public:
        Cylinder(double radius, double height);
        virtual ~Cylinder();
        
        virtual void init(const OdeHandle& odeHandle, double mass,
                        const VsgHandle& vsgHandle, char mode = Body | Geom | Draw);
        virtual void update();
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode();
        virtual void setMass(double mass, bool density = false);
        
    private:
        float radius;
        float height;
    };

    class Disk : public Primitive {
    public:
        Disk(double radius);
        virtual ~Disk();
        
        virtual void init(const OdeHandle& odeHandle, double mass,
                        const VsgHandle& vsgHandle, char mode = Body | Geom | Draw);
        virtual void update();
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode();
        virtual void setMass(double mass, bool density = false);
        
    private:
        float radius;
    };

    /** Ray primitive
        The actual visual representation can have different length than the
        ray object. This is specified by length.
        SetLength is an efficient way to change the length at runtime.
    */
    class Ray : public Primitive {
    public:
        /**
            @param range The range of the ODE ray
            @param thickness if thickness == 0 then a line is used as a visual representation
                            if thickness > 0 a box with thickness x thickness x length is used.
            @param length Initial display length of the ray geometry (visual only)
        */
        Ray(double range, float thickness, float length);
        virtual ~Ray();

        virtual void init(const OdeHandle& odeHandle, double mass,
                        const VsgHandle& vsgHandle,
                        char mode = Geom | Draw);

        void setLength(float len);

        /**
         * Sets the color of the ray based on its length
         * Useful for visualizing sensor readings - shorter rays (closer objects) will be more red
         * @param len Length to use for coloring (typically current measured length)
         */
        void setColorForLength(float len);

        /**
         * Get the current length of the ray visualization
         * @return Current length of the ray
         */
        double getLength() const;

        /**
         * Get the maximum range of the ray
         * @return Maximum range of the ray
         */
        double getRange() const;

        /**
         * Print debug information about the ray to standard output
         */
        void printDebugInfo() const;

        virtual void update();

        virtual void setMass(double mass, bool density = false);

        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode() { return transformNode; }

    protected:
        double range;
        float thickness;
        float length;
    };


    /** Mesh primitive
        Loads a mesh from a file using VSG and represents it in ODE.
    */
    class Mesh : public Primitive {
    public:
        Mesh(const std::string& filename, float scale);
        virtual ~Mesh();

        virtual void init(const OdeHandle& odeHandle, double mass,
                        const VsgHandle& vsgHandle,
                        char mode = Body | Geom | Draw) override;

        virtual void update() override;
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode() override { return transformNode; }

        virtual void setMass(double mass, bool density = false) override;

        virtual float getRadius();

        /** Sets the BoundingShape externally.
            If not set or not found, we default to a sphere bounding.
        */
        virtual void setBoundingShape(Primitive* boundingShape);

        virtual void setPose(const Pose& pose) override;

    protected:
        std::string filename;
        float scale;
        Primitive* boundshape;  
        Pose poseWithoutBodyAndGeom;

        vsg::ref_ptr<vsg::Node> meshNode; // The loaded mesh node
    };


    /**
     Primitive for transforming a geom in respect to a parent primitive.
    Uses ODE's GeomTransform and a MatrixTransform in VSG.
    */
    class Transform : public Primitive {
    public:
        Transform(Primitive* parent, Primitive* child, const Pose& pose, bool deleteChild = true);
        virtual ~Transform();

        virtual void init(const OdeHandle& odeHandle, double mass,
                        const VsgHandle& vsgHandle,
                        char mode = Body | Geom | Draw) override;

        virtual void update() override;
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode() override { return transformNode; }

        virtual void setMass(double mass, bool density = false) override;
        // setting the pose is not supported (as per original code)
        virtual void setPose(const Pose& pose) override {}
        Primitive* child;

    protected:
        Primitive* parent;
        Pose pose;
        bool deleteChild;
    };


    /**
     Dummy Primitive with no actual geometry or body.
    Can be used to represent static world or virtual objects.
    */
    class DummyPrimitive : public Primitive {
    public:
        DummyPrimitive() {
            body = 0;
            geom = 0;
        }

        virtual void init(const OdeHandle& odeHandle, double mass,
                        const VsgHandle& vsgHandle, char mode = Body | Geom | Draw) override {
            this->mode = mode;
            // No geometry or body is created.
        }

        virtual void update() override {}
        virtual vsg::ref_ptr<vsg::MatrixTransform> getTransformNode() override { return nullptr; }

        virtual void setMass(double mass, bool density = false) override {}

        virtual void setPosition(const Pos& pos) override {
            this->pos = pos;
        }
        virtual Pos getPosition() const override {
            return pos;
        }
        virtual void setPose(const Pose& pose) override {
            // just store translation since we have no real geom or body
            setPosition(pose.getTrans());
        }

        virtual Pos getVel() const override {
            return vel;
        }
        virtual void setVel(Pos vel) {
            this->vel = vel;
        }

    private:
        Pos vel;
        Pos pos;
    };


}

#endif // PRIMITIVE_H