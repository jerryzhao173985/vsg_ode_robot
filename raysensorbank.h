#ifndef __RAYSENSORBANK_H
#define __RAYSENSORBANK_H

#include <vector>
#include "raysensor.h"

namespace lpzrobots {

/** Class for a bank (collection) of ray sensors.
    Ray sensors can be registered at the bank. Methods for sensing
    and reading the sensor values of all sensors are provided.
 */
  class RaySensorBank  : public PhysicalSensor {
  public:
    RaySensorBank();

    virtual ~RaySensorBank();

    virtual void setInitData(const OdeHandle& odeHandle,
                             const VsgHandle& vsgHandle,
                             const vsg::dmat4& pose) override;

    // ---- Sensor interface -----
    virtual void init(Primitive* own, Joint* joint = 0) override;

    virtual int getSensorNumber() const override;

    virtual bool sense(const GlobalData& globaldata) override;

    virtual int get(sensor* sensors, int length) const override;
    virtual std::list<sensor> getList() const override;

    virtual void update() override;

    /** registers a new sensor at the sensor bank. The body and the pose have to be provided.
        @param raysensor RaySensor to add
        @param body body to which the sensor should be connected
        @param pose relative position/orientation
        @param range maximum sense range of the sensor
        @return index of the sensor
    */
    virtual unsigned int registerSensor(RaySensor* raysensor, Primitive* body,
                                        const vsg::dmat4& pose, float range,
                                        RaySensor::rayDrawMode drawMode);



    /// returns the number of sensors
    virtual int size() { return bank.size(); }


    /** set the range of the specified sensor (index)
        @param index index of sensor to modify
        @param range new length of the sensor
    */
    virtual void setRange(unsigned int index, float range);

    /** set the range of all sensors
        @param range new length of the sensors
    */
    virtual void setRange(float range);

    /** returns the spaceID of the sensor space
     */
    virtual dSpaceID getSpaceID();


    // delete all registered sensors.
    virtual void clear();

    // returns true if initialized
    virtual bool isInitialized() { return initialized;}

  protected:
    std::vector<RaySensor*> bank;
    bool initialized;
  };

}

#endif
