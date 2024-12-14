#ifndef __RAYSENSOR_H
#define __RAYSENSOR_H

#include <ode/common.h>
#include <ode/ode.h>
#include <cmath>
#include <assert.h>
#include "position.h"

#include "primitive.h"
#include "odehandle.h"
#include "physicalsensor.h"
#include "globaldata.h"

// #include <osg/Matrix>
// #include <osg/Vec3>
// #include "osgprimitive.h"
// #include "osgforwarddecl.h"

namespace lpzrobots {

  class Cylinder;
  class Box;
  class Transform;
  class Ray;

/** Class for Ray-based sensors.
    This are sensors which are based on distance measurements using the ODE geom class Ray.
    The sensor value is obtained by collisions.
    See also RaySensorBank, which is an object for managing multiple ray sensors.
 */
  class RaySensor : public PhysicalSensor {
  public:
    enum rayDrawMode { drawNothing, drawRay, drawSensor, drawAll};

    RaySensor();

    /**
     * @param size size of sensor in simulation
     * @param range maximum range of the Ray sensor
     * @param drawMode draw mode of the sensor
     */
    RaySensor(double size , double range, rayDrawMode drawMode);

    ~RaySensor();

    ///Create a copy of this without initialization
    virtual RaySensor* clone() const;

    void setPose(const vsg::dmat4& pose) override;

    void init(Primitive* own, Joint* joint = 0) override;

    bool sense(const GlobalData& globaldata) override;

    int get(sensor* sensors, int length) const override;

    std::list<sensor> getList() const override;

    virtual int getSensorNumber() const override;

    virtual void update() override;

    ///Set maximum range of ray
    virtual void setRange(double range);

    ///Set draw mode of ray
    virtual void setDrawMode(rayDrawMode drawMode);

    ///Set length of ray (needed for callback)
    void setLength(double len, long int time);

  protected:
    //Initialize variables of ray sensor. Should be called
    //by every constructor
    void defaultInit();

    double size; // size of graphical sensor
    double range; // max length
    rayDrawMode drawMode;

    double len;   // measured length
    double lastlen; //last measured length
    double detection;   // detected length (internally used)
    long lasttimeasked; // used to make sense return the same number if called two times in one timestep

    Cylinder* sensorBody;
    Transform* transform;
    Ray* ray;
    bool initialised;

  };

  // Callback for ray collisions
  int rayCollCallback(dSurfaceParameters& params, GlobalData& globaldata, void *userdata,
                      dContact* contacts, int numContacts,
                      dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2);

}

#endif
