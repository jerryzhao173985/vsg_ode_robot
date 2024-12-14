#ifndef __TORQUESENSOR_H
#define __TORQUESENSOR_H

#include "sensor.h"
#include <ode/ode.h>

namespace lpzrobots {

  class Joint;

  /** Class for sensing the torque that are applied to the joint by a motor.
      The sensor value can be interpreted as a motor current.
  */
  class TorqueSensor : public Sensor {
  public:

    /**
       @param maxtorque at this torque the sensor value is 1.
       @param avg number of averaging steps (def 1) (very noisy for universal joint)
     */
    TorqueSensor(double maxtorque = 1.0, int avg = 1);
    virtual ~TorqueSensor();

    /** the primitive is not required here, set it to NULL
        @param joint the joint on which to measure the torques.
    */
    virtual void init(Primitive* own, Joint* joint = 0);
    virtual int getSensorNumber() const;

    virtual bool sense(const GlobalData& globaldata);
    virtual std::list<sensor> getList() const;
    virtual int get(sensor* sensors, int length) const; // we implement this one because easier with averaging

  private:
    Joint* joint;
    double maxtorque;
    std::vector<sensor> values;
    double tau; // for averaging
  };


}

#endif
