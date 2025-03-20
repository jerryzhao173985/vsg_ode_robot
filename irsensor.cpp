#include "irsensor.h"
#include <cmath>
#include <algorithm>

namespace lpzrobots {

  IRSensor::IRSensor(double exponent, double size, double range, rayDrawMode drawMode)
  : RaySensor(size, range, drawMode) {
    this->exponent = exponent;
    value = 0;
    setBaseInfo(SensorMotorInfo("IR Sensor").changequantity(SensorMotorInfo::Distance)
                .changename("IR").changemin(0).changemax(1));
  }

  bool IRSensor::sense(const GlobalData& globaldata) {
    // Call parent class to do the actual ray sensing
    RaySensor::sense(globaldata);
    
    // Get the measured length
    double measuredLength = this->len;
    
    // Apply the IR sensor characteristic to convert distance to sensor value
    value = characteritic(measuredLength);
    
    // Update ray visualization if drawing is enabled
    if (ray) {
      // Get the Ray pointer from the transform's child
      Ray* rayPtr = dynamic_cast<Ray*>(transform->child);
      if (rayPtr) {
        // Update the ray's length visualization - we still want to show
        // the actual detection distance, not the IR reading
        rayPtr->setLength(measuredLength);
        
        // But we use the IR value for color - red for high values, green for low
        // This maps the IR characteristic to the visualization
        float normalizedValue = static_cast<float>(value);
        Color sensorColor(normalizedValue, 1.0f - normalizedValue, 0.0f);
        rayPtr->setColor(sensorColor);
      }
    }
    
    // If we have a sensor body, update its color to match the IR reading
    if (sensorBody) {
      float normalizedValue = static_cast<float>(value);
      sensorBody->setColor(Color(normalizedValue, 1.0f - normalizedValue, 0.0f));
      sensorBody->update();
    }
    
    return true;
  }

  double IRSensor::getValue() {
    return value;
  }

  int IRSensor::get(sensor* sensors, int length) const {
    assert(length > 0);
    sensors[0] = value;
    return 1;
  }

  std::list<sensor> IRSensor::getList() const {
    return {value};
  }

  double IRSensor::characteritic(double len) {
    // Convert length to IR characteristic value
    // IR sensors typically return higher values for closer objects
    // and lower values for distant objects
    
    // Normalize length to range [0,1]
    double normalizedLength = (range - len) / range;
    
    // Clamp to range [0,1]
    normalizedLength = std::max(0.0, std::min(1.0, normalizedLength));
    
    // Apply exponent for non-linear characteristic
    return pow(normalizedLength, exponent);
  }

}
