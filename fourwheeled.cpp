#include <ode/ode.h>
#include <assert.h>

#include "fourwheeled.h"
#include "joint.h"
#include "irsensor.h"
#include "primitive.h"

using namespace std;

namespace lpzrobots {
  // Global counter for monitoring obstacle avoidance activation
  static int g_avoidance_activations = 0;
  static int g_total_motor_calls = 0;

  FourWheeled::FourWheeled(const OdeHandle& odeHandle, const VsgHandle& vsgHandle,
                           FourWheeledConf conf, const std::string& name)
    : Nimm4(odeHandle, vsgHandle, name, conf.size, conf.force, conf.speed, conf.sphereWheels), conf(conf)
  {
    length=conf.size/2.0; // length of body

    wheelsubstance=conf.wheelSubstance;
  };


  FourWheeled::~FourWheeled(){
    destroy();
  }

  int FourWheeled::getSensorNumberIntern(){
    if(conf.twoWheelMode){
      assert(Nimm4::getSensorNumberIntern() == 4);
      return 2;
    }else
      return Nimm4::getSensorNumberIntern();
  }

  int FourWheeled::getSensorsIntern(double* sensors, int sensornumber){
    int len = 0;
    if(conf.twoWheelMode){
      sensor nimm4s[4];
      Nimm4::getSensorsIntern(nimm4s,4);
      sensors[len++] = (nimm4s[0]+nimm4s[2])/2;
      sensors[len++] = (nimm4s[1]+nimm4s[3])/2;
    } else {
      len = Nimm4::getSensorsIntern(sensors,sensornumber);
    }

    return len;
  };

  int FourWheeled::getMotorNumberIntern(){
    if(conf.twoWheelMode)
      return 2;
    else
      return Nimm4::getMotorNumberIntern();
  }

  void FourWheeled::setMotorsIntern(const double* motors, int motornumber){
    g_total_motor_calls++;
    
    // Create a copy of motor commands for potential modification
    std::vector<double> modifiedMotors(motors, motors + motornumber);
    bool avoidanceActivated = false;
    
    // Apply basic obstacle avoidance if IR sensors are enabled
    if (conf.irFront || conf.irSide || conf.irBack) {
      // Get current sensor values to check for obstacles
      int totalSensors = getSensorNumber();
      if (totalSensors > 4) {  // More than just wheel sensors
        std::vector<double> sensorValues(totalSensors);
        getSensors(sensorValues.data(), totalSensors);
        
        // IR sensors come after wheel sensors (indices 4+)
        // Sensor layout: [wheel0, wheel1, wheel2, wheel3, IR sensors...]
        double frontLeftIR = (totalSensors > 4) ? sensorValues[4] : 0.0;
        double frontRightIR = (totalSensors > 5) ? sensorValues[5] : 0.0;
        double rightSideIR = (totalSensors > 6) ? sensorValues[6] : 0.0;
        double leftSideIR = (totalSensors > 7) ? sensorValues[7] : 0.0;
        
        // Apply obstacle avoidance reflexes
        const double OBSTACLE_THRESHOLD = 0.7;  // Sensor value threshold for obstacle
        const double AVOIDANCE_FACTOR = 0.3;    // How much to reduce motor power
        
        // Front obstacle avoidance
        if (frontLeftIR > OBSTACLE_THRESHOLD || frontRightIR > OBSTACLE_THRESHOLD) {
          // Reduce forward motion when front obstacle detected
          for (int i = 0; i < motornumber; i++) {
            if (modifiedMotors[i] > 0) {  // Only reduce positive (forward) motion
              modifiedMotors[i] *= (1.0 - AVOIDANCE_FACTOR);
              avoidanceActivated = true;
            }
          }
        }
        
        // Side obstacle avoidance - create turning bias
        if (rightSideIR > OBSTACLE_THRESHOLD && motornumber >= 2) {
          // Turn left when right obstacle detected
          modifiedMotors[1] *= (1.0 - AVOIDANCE_FACTOR);  // Reduce right wheel
          avoidanceActivated = true;
        }
        if (leftSideIR > OBSTACLE_THRESHOLD && motornumber >= 2) {
          // Turn right when left obstacle detected  
          modifiedMotors[0] *= (1.0 - AVOIDANCE_FACTOR);  // Reduce left wheel
          avoidanceActivated = true;
        }
        
        if (avoidanceActivated) {
          g_avoidance_activations++;
        }
      }
    }
    
    // Apply the potentially modified motor commands
    if(conf.twoWheelMode){
      motor nimm4m[4];
      nimm4m[0] = modifiedMotors[0];
      nimm4m[2] = modifiedMotors[0];
      nimm4m[1] = (motornumber > 1) ? modifiedMotors[1] : modifiedMotors[0];
      nimm4m[3] = (motornumber > 1) ? modifiedMotors[1] : modifiedMotors[0];
      Nimm4::setMotorsIntern(nimm4m,4);
    }else {
      Nimm4::setMotorsIntern(modifiedMotors.data(), motornumber);
    }
  }

  // Function to get avoidance statistics
  std::pair<int, int> FourWheeled::getAvoidanceStats() {
    return std::make_pair(g_avoidance_activations, g_total_motor_calls);
  }


  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void FourWheeled::create(const vsg::dmat4& pose){
    Nimm4::create(pose);
    // create frame to not fall on back

    if(conf.useBumper){
      bumper = new Box(0.1 , width+2*wheelthickness+radius, length+0.7*width);
      // bumper->setTexture("Images/wood.rgb");
      bumpertrans = new Transform(objects[0], bumper,
                                  vsg::translate(width*0.6-radius, 0.0, 0.0));
      bumpertrans->init(odeHandle, 0, vsgHandle);
      objects.push_back(bumpertrans);
    }else if(conf.useButton){
      bumper = new Box(width*0.6 , width*0.7, 0.1);
      // bumper->setTexture("Images/wood.rgb");
      bumpertrans = new Transform(objects[0], bumper,
                                  vsg::translate(0.0,0.0, -length*0.9));
      bumpertrans->init(odeHandle, 0, vsgHandle.changeColor(1,1,0));
      objects.push_back(bumpertrans);
    }



    /* initialize sensorbank (for use of infrared sensors)
     * sensor values (if sensors used) are saved in the vector of
     * sensorvalues in the following order:
     * front left
     * front right
     * right middle
     * rear right
     * rear left
     * left  middle
    */
    if (conf.irFront || conf.irSide || conf.irBack) {
      // Create sensor bank using std::shared_ptr for automatic memory management
      auto irSensorBank = std::make_shared<RaySensorBank>();
      irSensorBank->setInitData(odeHandle, vsgHandle, vsg::translate(0.0,0.0,0.0));
      
      if (conf.irFront) { // add front left and front right infrared sensors
        for(int i=-1; i<2; i+=2){
          // Create an IR sensor with proper exponent (2.0) for realistic IR behavior
          IRSensor* sensor = new IRSensor(2.0, width/20.0, conf.irRangeFront, RaySensor::drawAll);
          // Position sensor at front of robot with appropriate angle
          irSensorBank->registerSensor(sensor, objects[0],
                            vsg::translate(0.0, -i*width/10, length/2 + width/2 - width/60) *
                            vsg::rotate(i*M_PI/10, vsg::dvec3(1,0,0)),
                            conf.irRangeFront, RaySensor::drawAll);
        }
      }
      
      if (conf.irSide) { // add right side and left side IR sensors
        // Right side sensor
        IRSensor* sensor = new IRSensor(2.0, width/20.0, conf.irRangeSide, RaySensor::drawAll);
        irSensorBank->registerSensor(sensor, objects[0],
                          vsg::translate(0.0, -width/2, 0.0) *
                          vsg::rotate(M_PI/2, vsg::dvec3(1,0,0)),
                          conf.irRangeSide, RaySensor::drawAll);
                          
        // Left side sensor
        sensor = new IRSensor(2.0, width/20.0, conf.irRangeSide, RaySensor::drawAll);
        irSensorBank->registerSensor(sensor, objects[0],
                          vsg::translate(0.0, width/2, 0.0) *
                          vsg::rotate(-M_PI/2, vsg::dvec3(1,0,0)), 
                          conf.irRangeSide, RaySensor::drawAll);
      }
      
      if (conf.irBack) { // add rear right and rear left infrared sensors
        for(int i=-1; i<2; i+=2){
          IRSensor* sensor = new IRSensor(2.0, width/20.0, conf.irRangeBack, RaySensor::drawAll);
          irSensorBank->registerSensor(sensor, objects[0],
                          vsg::translate(0.0, i*width/10, -(length/2 + width/2 - width/60)) *
                          vsg::rotate(i*M_PI, vsg::dvec3(0,1,0)) *
                          vsg::rotate(-i*M_PI/10, vsg::dvec3(1,0,0)),
                          conf.irRangeBack, RaySensor::drawAll);
        }
      }
      
      // Add the sensor bank to the robot
      addSensor(irSensorBank);
    }
  };


  // returns the joint with index i
  Joint* FourWheeled::getJoint(int i){
    if(i < 0 || i >= (int)joints.size()) {
        std::cerr << "FourWheeled::getJoint: Joint index " << i << " out of bounds (0-" 
                  << (joints.size()-1) << ")" << std::endl;
        // Return the first joint as a fallback if available
        return joints.empty() ? nullptr : joints[0];
    }
    return joints[i];
  }


  /** destroys vehicle and space
   */
  void FourWheeled::destroy(){
    Nimm4::destroy();
  }

}

