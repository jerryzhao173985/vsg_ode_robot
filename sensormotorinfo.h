#ifndef __SENSORMOTORINFO_H
#define __SENSORMOTORINFO_H

#include <string>
#include "types.h"


/**
 * Interface for objects, that can be stored and restored to/from a file stream (binary).
*/

class SensorMotorInfo {
public:
  enum Type { Continuous, Discrete, Binary };
  enum Quantity { Position, Velocity, Force, Distance, Other };

  SensorMotorInfo(std::string name=std::string())
    : name(name), min(-1.0), max(1.0), index(0), quantity(Position), type(Continuous)
  {}

  CHANGER( SensorMotorInfo, std::string, name);
  CHANGER( SensorMotorInfo, double, min);
  CHANGER( SensorMotorInfo, double, max);
  CHANGER( SensorMotorInfo, Quantity, quantity);
  CHANGER( SensorMotorInfo, Type, type);


  std::string name;
  double min;
  double max;
  int index; // index within one Sensor
  Quantity quantity;
  Type type;
};

#endif
