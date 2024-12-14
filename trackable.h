#ifndef __TRACKABLE_H
#define __TRACKABLE_H

#include "position.h"
#include "matrix.h"

namespace matrix {
  class Matrix;
}

/**
 * Abstract class (interface) for trackable objects (used for robots)
 *
 *
 */
class Trackable{
public:

  /**
   * Constructor
   */
  Trackable(){
  };

  virtual ~Trackable() {};

  /** returns name of trackable
   */
  virtual std::string getTrackableName() const =0;

  /** returns position of the object
      @return vector of position (x,y,z)
   */
  virtual Position getPosition() const =0;

  /** returns linear speed vector of the object
      @return vector  (vx,vy,vz)
   */
  virtual Position getSpeed() const =0;

  /** returns angular velocity vector of the object
      @return vector  (wx,wy,wz)
   */
  virtual Position getAngularSpeed() const =0;

  /** returns the orientation of the object
      @return 3x3 rotation matrix
   */
  virtual matrix::Matrix getOrientation() const =0;

};

#endif

