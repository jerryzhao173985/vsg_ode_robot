#ifndef __TMPPRIMITIVE_H
#define __TMPPRIMITIVE_H

#include "tmpobject.h"
#include "primitive.h"

namespace lpzrobots {

  class Primitive;
  class Joint;

  /**
   holding a temporary primitive
   */
  class TmpPrimitive : public TmpObject {
  public:
    /** creates a new item from the given primitives and initializes it.
        The lifetime is set when adding it to globalData
     */
    TmpPrimitive(Primitive* p, char mode, double mass, const Pose& pose,
                 const Color& color);

    /// provided for convenience to supply color as name and alpha independently
    TmpPrimitive(Primitive* p, char mode, double mass, const Pose& pose,
                 const std::string& colorname, float alpha = 1.0);

    virtual void init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle);
    virtual void deleteObject();
    virtual void update();

  private:
    Primitive* item;
    char mode;
    double mass;
    Pose pose;
    Color color;
    std::string colorname;
    bool useColorName;
    float alpha;
    bool initialized;
  };

  /**
   holding a temporary graphical item
   */
  class TmpDisplayItem : public TmpObject {
  public:
    /** creates a new item from the given primitives and initializes it.
        The lifetime is set when adding it to globalData
     */
    TmpDisplayItem(Primitive* p, const Pose& pose, const Color& color);

    /// provided for convenience to supply color as name and alpha independently
    TmpDisplayItem(Primitive* p, const Pose& pose,
                   const std::string& colorname, float alpha = 1.0);

    virtual void init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle);

    virtual void deleteObject();
    virtual void update() {} // nothing to be done here, because they do not move

  private:
    Primitive* item;
    Pose pose;
    Color color;
    std::string colorname;
    bool useColorName;
    float alpha;
    bool initialized;
  };

  /**
   holding a temporary joint
   */
  class TmpJoint : public TmpObject {
  public:
    /** creates a new tmporary object from the given joint and initializes it.
        The lifetime is set when adding it to globalData
     */
    TmpJoint(Joint* p, const Color& color, bool withVisual = true, double visualSize = 0.2,
             bool ignoreColl = true);

    /// provided for convenience to supply color as name and alpha independently
    TmpJoint(Joint* p, const std::string& colorname, float alpha = 1.0,
             bool withVisual = true, double visualSize = 0.2, bool ignoreColl = true);

    virtual void init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle);

    virtual void deleteObject();
    virtual void update();

  private:
    Joint* joint;
    Color color;
    std::string colorname;
    bool useColorName;
    float alpha;
    bool withVisual;
    double visualSize;
    bool ignoreColl;
    bool initialized;
  };

}

#endif
