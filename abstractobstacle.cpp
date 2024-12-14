#include "abstractobstacle.h"
#include "primitive.h"
#include "stl_adds.h"
#include "position.h"

#include "mathutils.h"
#include "pos.h"

using namespace std;

namespace lpzrobots {

  /*
   * Constructor
   * @param odeHandle containing ODE stuff like world, space and jointgroup
   * @param vsgHandle containing VSG stuff like scene, color...
   * be used for creation of obstacles
   */
  AbstractObstacle::AbstractObstacle(const OdeHandle& odeHandle, const VsgHandle& vsgHandle)
    : pose(vsg::translate(0.0,0.0,0.0)), odeHandle(odeHandle), vsgHandle(vsgHandle)
  {
    // initialize the pose matrix correctly
    obstacle_exists=false;
  };

  AbstractObstacle::~AbstractObstacle(){
    if(obstacle_exists) AbstractObstacle::destroy();
  }

  void AbstractObstacle::update(){
    if (obstacle_exists){
      for (unsigned int i=0; i<obst.size(); i++){
        if(obst[i]) obst[i]->update();
      }
    }
  };

  /*
   * sets position of the obstacle and creates/recreates obstacle if necessary
   */
  void AbstractObstacle::setPos(const vsg::dvec3& pos) {
    pose.setTrans(pos);
    setPose(pose);
  };

  /*
   * sets position of the obstacle and creates/recreates obstacle if necessary
   */
  void AbstractObstacle::setPosition(const vsg::dvec3& pos) {
    setPos(pos);
  };

  /*
   * gives actual position of the obstacle
   */
  vsg::dvec3 AbstractObstacle::getPos(){
    return pose.getTrans();
  }

  /*
   * gives actual pose of the obstacle
   */
  vsg::dmat4 AbstractObstacle::getPose(){ return pose; }

  void AbstractObstacle::setColor(const Color& color) {
    vsgHandle.color = color;
    if (obstacle_exists) {
      FOREACH(vector<Primitive*>, obst, it){
        (*it)->setColor(color);
      }
    }
  }

  // void AbstractObstacle::setColor(const string& color) {
  //   vsgHandle.color = vsgHandle.getColor(color);
  //   if (obstacle_exists) {
  //     FOREACH(vector<Primitive*>, obst, it){
  //       (*it)->setColor(color);
  //     }
  //   }
  // }

  // void AbstractObstacle::setTexture(const std::string& texturefilename){
  //   setTexture(0,TextureDescr(texturefilename,-1,-1));
  // }

  // void AbstractObstacle::setTexture(const TextureDescr& texture){
  //   setTexture(0,texture);
  // }

  // void AbstractObstacle::setTexture(int surface, const TextureDescr& texture){
  //   if(obstacle_exists){
  //     FOREACH( std::vector<Primitive*>, obst, o){
  //       if(*o) (*o)->setTexture(surface,texture);
  //     }
  //   }else{
  //     setTexture(0,surface,texture);
  //   }
  // }

  // void AbstractObstacle::setTexture(int primitive, int surface, const TextureDescr& texture){
  //   if(obstacle_exists){
  //     if(primitive < (signed)textures.size())
  //       obst[primitive]->setTexture(surface,texture);
  //   }else{
  //     if(primitive >= (signed)textures.size()) textures.resize(primitive+1);
  //     if(surface >= (signed)textures[primitive].size()) textures[primitive].resize(surface+1);
  //     textures[primitive][surface]=texture;
  //   }
  // }


  // TextureDescr AbstractObstacle::getTexture(int primitive, int surface) const{
  //   // take the last primitive we have texture information for.
  //   if(primitive >= (signed)textures.size())
  //     primitive = textures.size()-1;
  //   // take the last surface we have texture information for.
  //   if(surface >= (signed)textures[primitive].size()) surface = textures[primitive].size()-1;
  //   if(primitive<0 || surface<0) return TextureDescr();
  //   return textures[primitive][surface];
  // }

  // std::vector<TextureDescr> AbstractObstacle::getTextures(int primitive) const{
  //   // take the last primitive we have texture information for.
  //   if(primitive >= (signed)textures.size())
  //     primitive = textures.size()-1;
  //   // take the last surface we have texture information for.
  //   if(primitive<0) return std::vector<TextureDescr>();
  //   return textures[primitive];
  // }


  void AbstractObstacle::setSubstance(const Substance& substance){
    odeHandle.substance = substance;
    if (obstacle_exists) {
      FOREACH(vector<Primitive*>, obst, it){
        (*it)->substance=substance;
      }
    }
  }

  const Substance& AbstractObstacle::getSubstance(){
    return odeHandle.substance;
  }


  void AbstractObstacle::destroy(){
    FOREACH(vector<Primitive*>, obst, it){
      if(*it) delete(*it);
    }
    obst.clear();
    obstacle_exists=false;
  };

/** returns position of the object
@return vector of position (x,y,z)
  */
Position AbstractObstacle::getPosition() const {
  const Primitive* o = getMainPrimitive();

    // using the Geom has maybe the advantage to get the position of transform objects
    // (e.g. hand of muscledArm)
  if (o && o->getGeom())
    return Position(dGeomGetPosition(o->getGeom()));
  else if(o->getBody())
    return Position(dBodyGetPosition(o->getBody()));
  else return Position(0,0,0);
}

 Position AbstractObstacle::getSpeed() const {
  const Primitive* o = getMainPrimitive();
  if (o && o->getBody())
    return Position(dBodyGetLinearVel(o->getBody()));
  else return Position(0,0,0);
}

Position AbstractObstacle::getAngularSpeed() const {
  const Primitive* o = getMainPrimitive();
  if (o && o->getBody())
    return Position(dBodyGetAngularVel(o->getBody()));
  else return Position(0,0,0);
}

matrix::Matrix AbstractObstacle::getOrientation() const {
  const Primitive* o = getMainPrimitive();
  if (o && o->getBody()){
    return odeRto3x3RotationMatrix(dBodyGetRotation(o->getBody()));
  } else {
    matrix::Matrix R(3,3);
    return R^0; // identity
  }
}


}
