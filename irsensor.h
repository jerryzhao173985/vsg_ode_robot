#ifndef __IRSENSOR_H
#define __IRSENSOR_H

#include "raysensor.h"

namespace lpzrobots {

  /** Class for IR sensors.
      IR sensors are based on distance measurements using the ODE geom class Ray.
      The sensor value is obtained by collisions, which are handled by the simulation
      environement. The information of a collision comes to the sensor via the
      collision callback of the substance used for the ray (actually for the transform).
      If no collision is detected the value should be zero, so that a stamp is used.
      The length measured in this way is modified by the 'characteristic' of the IR sensor
  */
  class IRSensor : public RaySensor {
  public:
    /**
       @param exponent exponent of the sensor characteritic (default: 1 (linear))
       @param size size of sensor in simulation
       @param range maximum range of the IR sensor
       @param drawMode draw mode of the sensor
    */
    IRSensor(double exponent = 1, double size = 0.05, double range = 2,
             rayDrawMode drawMode = drawSensor);

    //Override sense to include characteristic
    virtual bool sense(const GlobalData& globaldata) override;

    //Override to return value given by characteristic
    virtual int get(sensor* sensors, int length) const override;
    virtual std::list<sensor> getList() const override;

    //Directly return value (needed for backward compatibility
    virtual double getValue();

    /// returns the exponent of the sensor characteritic (default: 1 (linear))
    virtual double getExponent () const { return exponent;}

    /// sets the exponent of the sensor characteritic (default: 1 (linear))
    virtual void setExponent (double exp) { exponent = exp;}

  protected:
    /** describes the sensor characteritic
        An exponential curve is used.
        @see setExponent()
    */
    virtual double characteritic(double len);

    double exponent; // exponent of the sensor characteritic

    double value; // actual sensor value
  };

}

#endif
