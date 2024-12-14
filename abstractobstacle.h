#ifndef __ABSTRACTOBSTACLE_H
#define __ABSTRACTOBSTACLE_H

#include <ode/ode.h>
#include "pose.h"
#include "odehandle.h"
#include "vsghandle.h"
#include <vsg/all.h>
#include <vector>

class Position;
namespace matrix { class Matrix; }

namespace lpzrobots {
  
class Primitive;

/**
 *  Abstract class (interface) for obstacles
 */
class AbstractObstacle{


 public:
  /**
   * Constructor
   * @param odeHandle containing ODE stuff like world, space and jointgroup
   * @param vsgHandle containing VSG stuff like scene, color...
   * be used for creation of obstacles
   */
  AbstractObstacle(const OdeHandle& odeHandle, const VsgHandle& vsgHandle);

  virtual ~AbstractObstacle();
  
  /**
   * updates the position if the scenegraph nodes
   * the default implementation calls update on all primitive on "obst"
   */
  virtual void update();
  
  /**
   * sets position of the obstacle and creates/recreates obstacle if necessary
   */
  virtual void setPos(const vsg::dvec3& pos);
  
    /**
   * sets position of the obstacle and creates/recreates obstacle if necessary
   */
  virtual void setPosition(const vsg::dvec3& pos);

  /**
   * gives actual position of the obstacle
   */
  virtual vsg::dvec3 getPos();

  /**
   * gives actual pose of the obstacle
   */
  virtual vsg::dmat4 getPose();

  /**
   * sets position of the obstacle and creates/recreates obstacle if necessary
   */
  virtual void setPose(const vsg::dmat4& pose) = 0;

  /**
   * sets the obstacle color
   * @param color values in RGBA
   */
  virtual void setColor(const Color& color);

  /*
   * sets the obstacle color from color name
   * @param color name of color in colorschema
   */
//   virtual void setColor(const std::string& color);



  /** assigns a texture to the all primitives of this obstactle with repeat -1,-1
      @see Primitive::setTexture()
  */
//   virtual void setTexture(const std::string& texturefilename);
//   /** assigns a texture to the all primitives of this obstactle
//       @see Primitive::setTexture()
//   */
//   virtual void setTexture(const TextureDescr& texture);
//   /** assigns a texture to the x-th surface of each primitive, 
//       @see Primitive::setTexture()
//   */
//   virtual void setTexture(int surface, const TextureDescr& texture);
//   /** assigns a texture to the x-th surface of the k-th primitive, (The texture setting of the 
//       last primitve is repeated for the remaining ones)
//       @see Primitive::setTexture()
//   */
//   virtual void setTexture(int primitive, int surface, const TextureDescr& texture);

//   /// returns the texture of the given surface on the given primitive
//   virtual TextureDescr getTexture(int primitive, int surface) const ;

//   /// returns the textures of the given primitive
//   virtual std::vector<TextureDescr> getTextures(int primitive) const;

  /// return the "main" primitive of the obtactle. The meaning of "main" is arbitrary
  virtual Primitive* getMainPrimitive() const = 0;

  /**
   * sets the substance of the obtactle. It is applied to all objects in obj
   * @param substance description of the substance
   */
  virtual void setSubstance(const Substance& substance);
  
  /// returns the substance of this obstacle 
  virtual const Substance& getSubstance();  
  
   /*********** BEGIN TRACKABLE INTERFACE *******************/
  
   /** returns position of the object
  @return vector of position (x,y,z)
   */
  virtual Position getPosition() const;
  
  /** returns linear speed vector of the object
  @return vector  (vx,vy,vz)
   */
  virtual Position getSpeed() const;
  
  /** returns angular velocity vector of the object
  @return vector  (wx,wy,wz)
   */
  virtual Position getAngularSpeed() const;
  
  /** returns the orientation of the object
  @return 3x3 rotation matrix
   */
  virtual matrix::Matrix getOrientation() const;
  
  /*********** END TRACKABLE INTERFACE *******************/

 protected:
  std::vector<Primitive*> obst; ///< primitives which belong to this obstacle
  
  // std::vector<std::vector<TextureDescr> > textures; ///< for each primitive the texture settings per surface

  Pose pose;
  bool obstacle_exists;

  OdeHandle odeHandle;
  VsgHandle vsgHandle; 
  

  /// is called to destroy the object. The default implementation is to delete all primitives in "obst". 
  virtual void destroy();

  /// overload this function to create the obstactle. All primitives should go into the list "obst"
  virtual void create()=0;

};

}

#endif
