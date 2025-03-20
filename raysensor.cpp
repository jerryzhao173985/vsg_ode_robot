#include "raysensor.h"

namespace lpzrobots {

  // This callback is triggered when the ray collides with an object.
  // userdata is the RaySensor pointer.
  // contacts[0].geom.depth is the length at which the collision was detected.
  int rayCollCallback(dSurfaceParameters& params, GlobalData& globaldata, void *userdata,
                      dContact* contacts, int numContacts,
                      dGeomID o1, dGeomID o2, const Substance& s1, const Substance& s2) {
    RaySensor* sensor = (RaySensor*)userdata;
    if (sensor) {
        // Set length based on collision depth
        sensor->setLength(contacts[0].geom.depth, globaldata.sim_step);
    }
    return 0; // we do not want to modify surface parameters here
  }

  void RaySensor::defaultInit(){
    initialised = false;
    len = 0;
    lastlen = -1;
    sensorBody = nullptr;
    ray = nullptr;
    transform = nullptr;
    detection = 1e5; // Very large value as default
    lasttimeasked = -1;
    setBaseInfo(SensorMotorInfo("Ray Sensor").changequantity(SensorMotorInfo::Distance).changemin(0));
  }

  RaySensor::RaySensor(){
    this->defaultInit();
    size = 0.05;
    range = 2.0;
    drawMode = drawNothing;
  }

  RaySensor::RaySensor(double size, double range, rayDrawMode drawMode)
    : size(size), range(range), drawMode(drawMode)
  {
    this->defaultInit();
  }

  RaySensor::~RaySensor(){
    // Transform owns the ray as its child if deleteChild=true was used.
    // If we didn't specify deleteChild in Transform, we must ensure deletion here.

    // By default, RaySensor creates a Transform with deleteChild=true, so transform 
    // will delete the ray automatically. We just need to delete transform and sensorBody.
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
    if (isInitDataSet) {
        w->setInitData(odeHandle, vsgHandle, pose);
    }
    return w;
  }

  int RaySensor::getSensorNumber() const{
    return 1; // only one ray measurement
  }

  void RaySensor::setPose(const vsg::dmat4& pose) {
    this->pose = pose;
    if (transform) {
        transform->setPose(Pose(pose));
    }
  }

  void RaySensor::init(Primitive* own, Joint* joint) {
    assert(isInitDataSet);
    
    // Set initial length to range (no detection)
    len = range;

    // Create ray with appropriate visualization
    ray = new Ray(range, 0.0f, range);

    // Create transform to position ray relative to parent
    transform = new Transform(own, ray, Pose(pose), true);

    // Configure mode based on draw settings
    char mode = Primitive::Geom;
    if(drawMode == drawAll || drawMode == drawRay) {
        mode |= Primitive::Draw;
    }

    // Initialize transform to position ray
    transform->init(odeHandle, 0, vsgHandle, mode);
    
    // Register collision callback to detect obstacles
    transform->substance.setCollisionCallback(rayCollCallback, this);

    // Create sensor body visualization if needed
    if (drawMode == drawAll || drawMode == drawSensor) {
        sensorBody = new Cylinder(size, size/5.0);
        sensorBody->init(odeHandle, 0, vsgHandle, Primitive::Draw);
    }

    update();  // Initial update to position everything
    initialised = true;
  }

  void RaySensor::setLength(double len, long int time) {
    // When a collision is detected, we store the minimum detection length
    detection = std::min(detection, len);
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
    if (globaldata.sim_step != lasttimeasked) {
        // we only update once per simulation step
        len = std::max(0.0, std::min(detection, range));
        detection = range; // reset detection for next step
        lasttimeasked = globaldata.sim_step;
        update(); // Update ray visualization
    }
    return true;
  }

  int RaySensor::get(sensor* sensors, int length) const {
    assert(length > 0);
    sensors[0] = len;
    return 1;
  }

  std::list<sensor> RaySensor::getList() const {
    return {len};
  }

  void RaySensor::update() {
    if (!initialised) return;

    // Always update the transform to make sure it follows parent
    if (transform) {
        transform->update();

        // Update ray length to match sensed value
        if (ray) {
            Ray* rayPtr = dynamic_cast<Ray*>(transform->child);
            if (rayPtr) {
                // Update ray visualization length
                rayPtr->setLength(len);
                
                // Color the ray based on length (red=close, yellow=far)
                rayPtr->setColorForLength(len);
            }
        }

        // Update sensor body position if present
        if (sensorBody) {
            // Get current ray transform
            Pose rayPose = transform->getPose();
            
            // Position sensor body at the ray origin
            sensorBody->setPose(rayPose);
            
            // Set color based on detection (red = close object, green = no detection)
            if (len < range) {
                // Closer = more red
                float intensity = 1.0f - (len / range);
                sensorBody->setColor(Color(1.0f, 1.0f - intensity, 0.0f));
            } else {
                // No detection = green
                sensorBody->setColor(Color(0.0f, 1.0f, 0.0f));
            }
            
            sensorBody->update();
        }
    }
  }

}
