#include "tmpprimitive.h"
#include "primitive.h"
#include "joint.h"

namespace lpzrobots {

  // ////////////////// TmpPrimitive //////////////////// //

  TmpPrimitive::TmpPrimitive(Primitive* p, char mode, double mass, const Pose& pose,
                             const Color& color)
    : item(p), mode(mode), mass(mass), pose(pose), color(color), alpha(1.0), initialized(false)
  {
    useColorName=false;
    if(!item)
      item = new Sphere(0.1);
  }

  TmpPrimitive::TmpPrimitive(Primitive* p, char mode, double mass, const Pose& pose,
               const std::string& colorname, float alpha)
    : item(p), mode(mode), mass(mass), pose(pose),
      colorname(colorname), alpha(alpha), initialized(false)
  {
    useColorName=true;
    if(!item)
      item = new Sphere(0.1);
  }

  void TmpPrimitive::init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle){
    Color mcolor(color);
    if(useColorName){
      mcolor = vsgHandle.getColor(colorname);
      mcolor.alpha() = alpha;
    }
    item->init(odeHandle, mass, vsgHandle.changeColor(mcolor), mode);
    item->setPose(pose);
    initialized=true;
  }

  void TmpPrimitive::deleteObject(){
    if(item) delete item;
    item=0;
  }

  void TmpPrimitive::update(){
    if(item) item->update();
  }


  // ////////////////// TmpDisplayItem //////////////////// //

  TmpDisplayItem::TmpDisplayItem(Primitive* p, const Pose& pose, const Color& color)
    : item(p), pose(pose), color(color), alpha(1.0), initialized(false)
  {
    useColorName=false;
    if(!item)
      item = new Sphere(0.1);
  }

  TmpDisplayItem::TmpDisplayItem(Primitive* p, const Pose& pose,
                                 const std::string& colorname, float alpha)
    : item(p), pose(pose), colorname(colorname), alpha(alpha), initialized(false)
  {
    useColorName=true;
    if(!item)
      item = new Sphere(0.1);
  }

  void TmpDisplayItem::init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle){
    Color mcolor(color);
    if(useColorName){
      mcolor = vsgHandle.getColor(colorname);
      mcolor.alpha() = alpha;
    }
    // initialzing mode as only draw the sphere graphical without geom or body
    // no need for mass, no need for odeHandle with Primitive::Draw
    item->init(odeHandle, 0, vsgHandle.changeColor(mcolor), Primitive::Draw);
    item->setMatrix(pose);
    initialized=true;
  }

  void TmpDisplayItem::deleteObject(){
    if(item) delete item;
    item=0;
  }


  TmpJoint::TmpJoint(Joint* p, const Color& color, bool withVisual, double visualSize,
                     bool ignoreColl)

    : joint(p), color(color), withVisual(withVisual), visualSize(visualSize),
      ignoreColl(ignoreColl), initialized(false)
  {
    useColorName=false;
  }

  TmpJoint::TmpJoint(Joint* p, const std::string& colorname, float alpha,
                     bool withVisual, double visualSize, bool ignoreColl)

    : joint(p), colorname(colorname), alpha(alpha),
      withVisual(withVisual), visualSize(visualSize),
      ignoreColl(ignoreColl), initialized(false)
  {
    useColorName=true;
  }

  void TmpJoint::init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle){
    Color mcolor(color);
    if(useColorName){
      mcolor = vsgHandle.getColor(colorname);
      mcolor.alpha() = alpha;
    }
    joint->init(odeHandle, vsgHandle.changeColor(mcolor),
                withVisual, visualSize, ignoreColl);
    initialized=true;
  }

  void TmpJoint::deleteObject(){
    if(joint) delete joint;
    joint=0;
  }

  void TmpJoint::update(){
    if(joint) joint->update();
  }


}

