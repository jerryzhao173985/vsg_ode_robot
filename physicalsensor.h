#ifndef __PHYSICALSENSOR_H
#define __PHYSICALSENSOR_H

#include "sensor.h"
#include "odehandle.h"
#include "vsghandle.h"

namespace lpzrobots {

  /** Abstract class for sensors that have a physical representation
  */
  class PhysicalSensor : public Sensor {
  public:

    PhysicalSensor() : isInitDataSet(false) {}
    virtual ~PhysicalSensor() {}

    /** sets the initial data structures
        @param pose position and orientation of sensor (e.g. camera) wrt.
        the primitive that is given at init()
     */
    virtual void setInitData(const OdeHandle& odeHandle,
                             const VsgHandle& vsgHandle,
                             const vsg::dmat4& pose) {
      this->odeHandle = odeHandle;
      this->vsgHandle = vsgHandle;
      this->pose      = pose;
      isInitDataSet   = true;
    }

    /// changes the relative pose of the sensor
    virtual void setPose(const vsg::dmat4& pose) { this->pose= pose; };

    /// relative pose of the sensor
    virtual vsg::dmat4 getPose() { return pose; };


  protected:
    OdeHandle odeHandle;
    VsgHandle vsgHandle;
    vsg::dmat4 pose;
    bool isInitDataSet;
  };
}

#endif
