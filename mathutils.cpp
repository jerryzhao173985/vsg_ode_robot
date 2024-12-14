#include <cmath>
#include "mathutils.h"
#include "axis.h"

using namespace matrix;

namespace lpzrobots {

  matrix::Matrix odeRto3x3RotationMatrixT ( const double R[12] ) {
    matrix::Matrix matrix(3,3);
    matrix.val(0,0)=R[0];
    matrix.val(0,1)=R[4];
    matrix.val(0,2)=R[8];
    matrix.val(1,0)=R[1];
    matrix.val(1,1)=R[5];
    matrix.val(1,2)=R[9];
    matrix.val(2,0)=R[2];
    matrix.val(2,1)=R[6];
    matrix.val(2,2)=R[10];
    return matrix;
  }

  matrix::Matrix odeRto3x3RotationMatrix ( const double R[12] ) {
    matrix::Matrix matrix(3,3);
    matrix.val(0,0)=R[0];
    matrix.val(1,0)=R[4];
    matrix.val(2,0)=R[8];
    matrix.val(0,1)=R[1];
    matrix.val(1,1)=R[5];
    matrix.val(2,1)=R[9];
    matrix.val(0,2)=R[2];
    matrix.val(1,2)=R[6];
    matrix.val(2,2)=R[10];
    return matrix;
  }

  /******************************************************************************/


  Position multMatrixPosition(const Matrix& r, Position& p){
    assert(r.getM()==3 && r.getN()==3);
    Matrix pm(3,1,p.toArray());
    Matrix rv = r*pm;
    return Position(rv.val(0,0), rv.val(1,0), rv.val(2,0));
  }

  /*
   * returns a rotation matrix for a rotation in x, y plane about the given angle
   */
  Matrix getRotationMatrix(const double& angle) {
    double data[16]={cos(angle),sin(angle),0,0,
                     -sin(angle),cos(angle),0,0,
                     0,0,1,0,
                     0,0,0,1};
    return Matrix(4,4,data);
  }

  /*
   * returns a translation matrix with the given Position
   */
  Matrix getTranslationMatrix(const Position& p) {
    double data[16]={0,0,0,p.x,
                     0,0,0,p.y,
                     0,0,1,p.z,
                     0,0,0,1};
    return Matrix(4,4,data);
  }

  /*
   * removes the translation in the matrix
   */
  Matrix removeTranslationInMatrix(const Matrix& pose){
    Matrix t(pose);
    // remove the three last values of the column 3
    t.val(0,3)=0.0f;
    t.val(1,3)=0.0f;
    t.val(2,3)=0.0f;
    return t;
  }

  /*
   * removes the rotation in the matrix
   */
  Matrix removeRotationInMatrix(const Matrix& pose){
    Matrix t(pose);
    t.val(0,0)=1.0f; t.val(0,1)=0.0f;
    t.val(1,0)=0.0f; t.val(1,1)=1.0f;
    return t;
  }

  /*
   * returns the angle between two vectors
   */
  double getAngle(Position a, Position b) {
    Matrix p(3,1,a.toArray()); // row wise
    Matrix q(1,3,b.toArray()); // column wise
    return acos((p * q).val(0,0) / (a.length()*b.length()) );
  }

}
