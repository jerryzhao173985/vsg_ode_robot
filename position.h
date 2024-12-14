#ifndef __POSITION_H
#define __POSITION_H

#include <iostream>
#include <cmath>

class Position
{
public:
  Position(){x=y=z=0; array[0]=array[1]=array[2]=0;}
  Position(double _x, double _y, double _z){ x=_x; y=_y; z=_z; }
  ///  p MUST have a size of at least 3 
  Position(const double* p){ x=p[0]; y=p[1]; z=p[2]; } 
  const double* toArray(){ array[0]=x;array[1]=y; array[2]=z; return array; } 
  Position operator+(const Position& sum) const 
           { Position rv(x+sum.x, y+sum.y, z+sum.z); return rv; }
  Position operator-(const Position& sum) const
           { Position rv(x-sum.x, y-sum.y, z-sum.z); return rv; }
  Position operator*(double f) const { Position rv(x*f, y*f, z*f); return rv; }

  double length() { return sqrt(x*x+y*y+z*z);  }

  double x;
  double y;
  double z;
  
  void print(){
    std::cout << '(' << x << ',' << y << ',' << z << ')' << std::endl;
  }
  
private:
  double array[3];
};

#endif
