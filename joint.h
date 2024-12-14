#ifndef __JOINT_H
#define __JOINT_H

#include <assert.h>
#include <list>
#include <vsg/all.h>
#include <ode/ode.h>

#include "axis.h"
#include "pos.h"
#include "odehandle.h"
#include "vsghandle.h"
#include "primitive.h"

namespace lpzrobots {

  /**
   * Joint is a base class that connects two Primitives with an ODE joint and optionally provides
   * a visual representation. Joints have anchors and axes. Some have one axis (e.g. Hinge),
   * others two (e.g. Universal), some none (e.g. Fixed, Ball).
   *
   * A Joint is typically constructed with two Primitives and an anchor position. The anchor defines
   * where in space the joint will be created. The joint must be initialized with init(...) before use.
   */
  class Joint {
  public:
    Joint(Primitive* part1, Primitive* part2, const vsg::dvec3& anchor)
      : joint(0), part1(part1), part2(part2), anchor(anchor), feedback(0) {
      assert(part1 && part2);
    }

    virtual ~Joint();

    /** 
     * Initializes (and creates) the joint in ODE. If withVisual is true, a small visual representation
     * of the joint (e.g. a sphere) is drawn. visualSize determines the size of this representation.
     * If ignoreColl is true, the pair of connected parts is ignored during collision handling.
     */
    virtual void init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                      bool withVisual = true, double visualSize = 0.2,
                      bool ignoreColl = true);

    /**
     * Updates the VSG nodes (if any) to match ODE transforms.
     * Typically called each simulation step after ODE steps have been performed.
     */
    virtual void update() = 0;

    /// Sets the ODE joint parameter (see ODE manual)
    virtual void setParam(int parameter, double value) = 0;

    /// Returns the ODE joint parameter (see ODE manual)
    virtual double getParam(int parameter) const = 0;

    /// Returns the underlying ODE joint ID
    dJointID getJoint() const  { return joint; }

    /// Returns the first connected primitive
    const Primitive* getPart1() const { return part1; }
    Primitive* getPart1() { return part1; }

    /// Returns the second connected primitive
    const Primitive* getPart2() const { return part2; }
    Primitive* getPart2() { return part2; }

    /// Returns the anchor position (in global coordinates)
    const vsg::dvec3& getAnchor() const { return anchor; }

    /// Returns the number of joint axes
    virtual int getNumberAxes() const = 0;

    /// Returns the nth axis of the joint (starting with 0). Default returns a zero axis.
    virtual Axis getAxis(int n) const { return Axis(); }

    /// Returns the positions of all axes (e.g. angles for hinge joints)
    virtual std::list<double> getPositions() const { return std::list<double>(); }

    /// Returns the rates of all axes (e.g. angular velocities)
    virtual std::list<double> getPositionRates() const { return std::list<double>(); }

    /// Writes the positions of all axes into sensorarray
    virtual int getPositions(double* sensorarray) const { return 0; }

    /// Writes the rates of all axes into sensorarray
    virtual int getPositionRates(double* sensorarray) const { return 0; }

    /// Enables or disables feedback mode (torque/force feedback)
    virtual void setFeedBackMode(bool mode);

    /// Returns the torque applied to part1 and part2 (if feedback is enabled)
    virtual bool getTorqueFeedback(Pos& t1, Pos& t2) const;

    /// Returns the force applied to part1 and part2 (if feedback is enabled)
    virtual bool getForceFeedback(Pos& f1, Pos& f2) const;

    /// Helper to create a pose matrix from anchor and axis
    static vsg::dmat4 anchorAxisPose(const vsg::dvec3& anchor, const Axis& axis);

  protected:
    dJointID joint;
    Primitive* part1;
    Primitive* part2;
    vsg::dvec3 anchor;

    dJointFeedback* feedback;

  public:
    OdeHandle odeHandle; // for access to ODE world, space, etc.
  };


  /**
   * OneAxisJoint defines an interface for joints with a single axis (e.g. Hinge, Slider).
   * It provides common methods for accessing and controlling that axis.
   */
  class OneAxisJoint : public Joint {
  public:
    OneAxisJoint(Primitive* part1, Primitive* part2, const vsg::dvec3& anchor, const Axis axis1)
      : Joint(part1, part2, anchor), axis1(axis1) {}

    virtual Axis getAxis(int n) const { 
      return (n==0) ? axis1 : Axis(); 
    }
    virtual Axis getAxis1() const { return axis1; }

    virtual double getPosition1() const = 0;
    virtual double getPosition1Rate() const = 0;
    virtual void addForce1(double force) = 0;

    virtual int getNumberAxes() const { return 1; }

    virtual std::list<double> getPositions() const;
    virtual std::list<double> getPositionRates() const;
    virtual int getPositions(double* sensorarray) const;
    virtual int getPositionRates(double* sensorarray) const;

  protected:
    Axis axis1;
  };


  /**
   * TwoAxisJoint defines an interface for joints with two axes (e.g. Universal, Hinge2).
   * It extends OneAxisJoint by adding a second axis.
   */
  class TwoAxisJoint : public OneAxisJoint {
  public:
    TwoAxisJoint(Primitive* part1, Primitive* part2, const vsg::dvec3& anchor, const Axis axis1,
                 const Axis axis2 )
      : OneAxisJoint(part1, part2, anchor, axis1), axis2(axis2) {}

    virtual Axis getAxis(int n) const { 
      if (n==0) return axis1; 
      else if (n==1) return axis2; 
      return Axis(); 
    }
    virtual Axis getAxis2() const { return axis2; }

    virtual double getPosition2() const = 0;
    virtual double getPosition2Rate() const = 0;
    virtual void addForce2(double force) = 0;

    void addForces(double force1,double force2){
      addForce1(force1); addForce2(force2);
    }

    virtual int getNumberAxes() const { return 2; }

    virtual std::list<double> getPositions() const;
    virtual std::list<double> getPositionRates() const;
    virtual int getPositions(double* sensorarray) const;
    virtual int getPositionRates(double* sensorarray) const;

  protected:
    Axis  axis2;
  };


  class FixedJoint : public Joint {
  public:
    FixedJoint(Primitive* part1, Primitive* part2,
               const vsg::dvec3& anchor = vsg::dvec3(0.0,0.0,0.0));
    virtual ~FixedJoint();

    virtual void init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                      bool withVisual = true, double visualSize = 0.2,
                      bool ignoreColl = true);
    virtual void update();
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter) const;

    virtual int getNumberAxes() const { return 0; }

  protected:
    Primitive* visual;
  };


  class HingeJoint : public OneAxisJoint {
  public:
    HingeJoint(Primitive* part1, Primitive* part2, const vsg::dvec3& anchor,
               const Axis& axis1);
    virtual ~HingeJoint();

    virtual void init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                      bool withVisual = true, double visualSize = 0.2,
                      bool ignoreColl = true);

    virtual void update();

    virtual void addForce1(double t);
    virtual double getPosition1() const;
    virtual double getPosition1Rate() const;
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter) const;

  protected:
    Primitive* visual;
  };


  class Hinge2Joint : public TwoAxisJoint {
  public:
    Hinge2Joint(Primitive* part1, Primitive* part2, const vsg::dvec3& anchor,
                 const Axis& axis1, const Axis& axis2);
    virtual ~Hinge2Joint();

    virtual void init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                      bool withVisual = true, double visualSize = 0.2,
                      bool ignoreColl = true);

    virtual void update();

    virtual void addForce1(double t1);
    virtual void addForce2(double t2);
    virtual double getPosition1() const;
    virtual double getPosition2() const;
    virtual double getPosition1Rate() const;
    virtual double getPosition2Rate() const;
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter) const;

  protected:
    Primitive* visual;
  };


  class UniversalJoint : public TwoAxisJoint {
  public:
    UniversalJoint(Primitive* part1, Primitive* part2, const vsg::dvec3& anchor,
                   const Axis& axis1, const Axis& axis2);
    virtual ~UniversalJoint();

    virtual void init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                      bool withVisual = true, double visualSize = 0.2,
                      bool ignoreColl = true);

    virtual void update();

    virtual void addForce1(double t1);
    virtual void addForce2(double t2);
    virtual double getPosition1() const;
    virtual double getPosition2() const;
    virtual double getPosition1Rate() const;
    virtual double getPosition2Rate() const;

    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter) const;

  protected:
    Primitive* visual1;
    Primitive* visual2;
  };


  class BallJoint : public Joint {
  public:
    BallJoint(Primitive* part1, Primitive* part2, const vsg::dvec3& anchor);
    virtual ~BallJoint();

    virtual void init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                      bool withVisual = true, double visualSize = 0.2,
                      bool ignoreColl = true);

    virtual void update();

    virtual int getNumberAxes() const { return 0; }
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter) const;

  protected:
    Primitive* visual;
  };


  class SliderJoint : public OneAxisJoint {
  public:
    SliderJoint(Primitive* part1, Primitive* part2, const vsg::dvec3& anchor,
                const Axis& axis1);
    virtual ~SliderJoint();

    virtual void init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                      bool withVisual = true, double visualSize = 0.1,
                      bool ignoreColl = true);

    virtual void update();

    virtual void addForce1(double t);
    virtual double getPosition1() const;
    virtual double getPosition1Rate() const;
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter) const;

  protected:
    Primitive* visual;
    double visualSize;
    VsgHandle vsgHandle;
  };

}
#endif
