#include "raysensor.h"

namespace lpzrobots {

  // This callback is triggered when the ray collides with an object.
  // userdata is the RaySensor pointer.
  // contacts[0].geom.depth is the length at which the collision was detected.
  int rayCollCallback(dSurfaceParameters& params, GlobalData& globaldata, void *userdata,
                      dContact* contacts, int numContacts,
                      dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2) {
    RaySensor* sensor = (RaySensor*)userdata;
    sensor->setLength(contacts[0].geom.depth, globaldata.sim_step);
    return 0; // we do not want to modify surface parameters here
  }

  void RaySensor::defaultInit(){
    initialised = false;
    len=0;
    lastlen=-1;
    sensorBody = 0;
    ray=0;
    transform=0;
    detection=1e5;
    lasttimeasked=-1;
    setBaseInfo(SensorMotorInfo("Ray Sensor").changequantity(SensorMotorInfo::Distance).changemin(0));
  }

  RaySensor::RaySensor(){
    this->defaultInit();
    // size = 0.0;
    // range = 0.0;
    // drawMode = drawNothing;
  }

  RaySensor::RaySensor(double size , double range, rayDrawMode drawMode)
    : size(size), range(range), drawMode(drawMode)
  {
    this->defaultInit();
  }

  RaySensor::~RaySensor(){
    // If we created them via new, we delete them here
    // transform owns the ray as its child if deleteChild=true was used.
    // If we didn't specify deleteChild in Transform, we must ensure deletion here.

    // By default, RaySensor creates a Transform with deleteChild=true, so transform 
    // will delete the ray automatically. We just need to delete transform and sensorBody if present.
    if (transform) {
      delete transform;
      transform = nullptr;
    }
    if (sensorBody) {
      delete sensorBody;
      sensorBody = nullptr;
    }
  }

  RaySensor* RaySensor::clone() const {
    RaySensor* w = new RaySensor(size, range, drawMode);
    w->setInitData(odeHandle, vsgHandle, pose);
    return w;
  }

  int RaySensor::getSensorNumber() const{
    return 1; // only one ray measurement
  }

  void RaySensor::setPose(const vsg::dmat4& pose) {
    this->pose = pose;
    if (transform) {
      // If transform is present, it places the ray relative to its parent
      // The transform sets child's pose relative to parent
      // In theory, we could call transform->child->setPose(pose), but transform sets child's pose once.
      // To keep consistent with original code: 
      // The original code asserted not implemented. We can implement now:
      transform->child->setPose(pose);
    }
  }

  void RaySensor::init(Primitive* own, Joint* joint) {
    assert(isInitDataSet);
    len = range;

    // Create ray: thickness=0 means we draw just a line
    // The ray extends along the local Z-axis. The length is visual only; the ODE ray range is set separately.
    ray = new Ray(range, 0.0f /*0.005*/, len);

    // Transform that places ray relative to 'own' primitive
    transform = new Transform(own, ray, pose, true /*deleteChild*/);

    // If we draw the ray or not depends on drawMode
    char mode = Primitive::Geom;
    if(drawMode == drawAll || drawMode == drawRay) {
      mode |= Primitive::Draw;
    }

    transform->init(odeHandle, 0, vsgHandle, mode);
    // Set collision callback on transform's substance
    transform->substance.setCollisionCallback(rayCollCallback, this);

    // If we also want to draw a sensor body (a cylinder representing the sensor device)
    switch(drawMode) {
      case drawAll:
      case drawSensor:
        // Create a cylinder representing the sensor device itself
        sensorBody = new Cylinder(size, size/5.0);
        sensorBody->init(odeHandle, 0, vsgHandle, Primitive::Draw);
        break;
      default:
        break;
    }

    update();
    initialised = true;
  }

  void RaySensor::setLength(double len, long int time) {
    // When a collision is detected, we store the minimum detection length
    detection = std::min(this->detection, len);
  }

  void RaySensor::setRange(double range) {
    assert(!initialised);
    this->range = range;
  }

  void RaySensor::setDrawMode(rayDrawMode drawMode) {
    assert(!initialised);
    this->drawMode = drawMode;
  }

  bool RaySensor::sense(const GlobalData& globaldata) {
    if(globaldata.sim_step != lasttimeasked) {
      // we only update once per simulation step
      len = std::max(0.0, std::min(detection, range));
      detection = range; // reset detection for next step
      lasttimeasked = globaldata.sim_step;
    }
    return true;
  }

  int RaySensor::get(sensor* sensors, int length) const {
    assert(length>0);
    sensors[0] = len;
    return 1;
  }

  std::list<sensor> RaySensor::getList() const {
    return {len};
  }

  void RaySensor::update() {
    if (!initialised) return;

    if (len != lastlen) {
      // Update the ray visualization
      ray->setLength(len);
      // Color the ray: red intensity based on length
      ray->setColor(Color(len*1.5, 0.0, 0.0));

      if(sensorBody) {
        // Sensor body color also changes with length
        sensorBody->setColor(Color(len*2.0, 0.0, 0.0));
      }
      lastlen = len;
    }

    // Update the ray
    ray->update();

    if(sensorBody) {
      // Position the sensor body slightly offset if needed
      // We move it slightly along z so it's not hidden by the ray line
      // The final position should be parent's pose * this sensor's pose * offset
      // The transform places the ray correctly relative to 'own'.
      // The sensorBody can be placed at the same position as the transform or slightly offset.
      // We'll place it at the start of the ray. The ray center is at 0,0,(length/2)
      // If we want the sensor body at the ray start (and ray extends forward),
      // Actually, the ray is centered around its origin and extends ±length/2.
      // We want the sensor body at the ray start (where the parent is), so offset by a small amount along z.
      Pose bodyPose = vsg::translate(0.0,0.0,0.005) * ray->getPose() * transform->getPose();
      sensorBody->setMatrix(bodyPose.getMatrix());
    }
  }

}
