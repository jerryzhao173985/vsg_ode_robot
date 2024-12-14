#include "irsensor.h"

namespace lpzrobots {

  IRSensor::IRSensor(double exponent, double size, double range, rayDrawMode drawMode)
  : RaySensor(size, range, drawMode) {

    this->exponent = exponent;
    value = 0;
  }


  bool IRSensor::sense(const GlobalData& globaldata){
    RaySensor::sense(globaldata);
    value = characteritic(len);
    return true;
  }

  double IRSensor::getValue(){
    return value;
  }

  int IRSensor::get(sensor* sensors, int length) const {
    assert(length>0);
    sensors[0]=value;
    return 1;
  }

  std::list<sensor> IRSensor::getList() const {
    return {value};
  }

  double IRSensor::characteritic(double len){
    double v = (range - len)/range;
    return v < 0 ? 0 : pow(v, exponent);
  }

}
