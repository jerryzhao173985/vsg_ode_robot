#ifndef __MATHUTILS_H
#define __MATHUTILS_H

#include "matrix.h"
#include "position.h"

namespace lpzrobots {

  class Axis;

/*   template<typename T> */
/*   inline T clip(T v,T minimum, T maximum) */
/*     { return clampBelow(clampAbove(v,minimum),maximum); } */

/*template<typename T> */
/*        inline T abs(T v) */
/*{ return ((v>0)?v:-v); } */

  template<typename T>
  inline T normalize360(T v)
    { while (v>360) v-=360; while (v<360) v+=360; return v; }
  /*******************************************************************************/

  /// converts an ode rotation matrix into a 3x3 rotation matrix (matrixlib)
  matrix::Matrix odeRto3x3RotationMatrixT ( const double R[12] );

  /// converts an ode rotation matrix into a 3x3 rotation matrix (matrixlib)
  matrix::Matrix odeRto3x3RotationMatrix ( const double R[12] );

  /*******************************************************************************/

  /**
     Multiplies 3x3 matrix with position
  */
  Position multMatrixPosition(const matrix::Matrix& r, Position& p);

  /**
   * returns a rotation matrix with the given angle
   */
  matrix::Matrix getRotationMatrix(const double& angle);


  /**
   * returns a translation matrix with the given Position
   */
  matrix::Matrix getTranslationMatrix(const Position& p) ;


  /**
   * removes the translation in the matrix
   */
  matrix::Matrix removeTranslationInMatrix(const matrix::Matrix& pose);


  /**
   * removes the rotation in the matrix
   */
  matrix::Matrix removeRotationInMatrix(const matrix::Matrix& pose) ;


  /**
   * returns the angle between two vectors
   */
  double getAngle(Position a, Position b) ;

}

#endif
