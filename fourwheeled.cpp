#include <ode/ode.h>
#include <assert.h>

#include "fourwheeled.h"
#include "joint.h"
#include "irsensor.h"
#include "primitive.h"

using namespace std;

namespace lpzrobots {

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
    if(conf.twoWheelMode){
      motor nimm4m[4];
      nimm4m[0] = motors[0];
      nimm4m[2] = motors[0];
      nimm4m[1] = motors[1];
      nimm4m[3] = motors[1];
      Nimm4::setMotorsIntern(nimm4m,4);
    }else
       Nimm4::setMotorsIntern(motors,motornumber);

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
    RaySensorBank* irSensorBank = new RaySensorBank();
    irSensorBank->setInitData(odeHandle, vsgHandle, vsg::translate(0.0,0.0,0.0));
    if (conf.irFront){ // add front left and front right infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
        IRSensor* sensor = new IRSensor();
        irSensorBank->registerSensor(sensor, objects[0],
                                    vsg::translate(0.0,-i*width/10,length/2 + width/2 - width/60 ) *
                                    vsg::rotate(i*M_PI/10, vsg::dvec3(1,0,0)),
                                    conf.irRangeFront, RaySensor::drawAll);
      }
    }
    if (conf.irSide){ // add right infrared sensor to sensorbank if required
      IRSensor* sensor = new IRSensor();
      irSensorBank->registerSensor(sensor, objects[0],
                                  //vsg::rotate(i*M_PI/2, vsg::dvec3(0,0,1)) *
                                  vsg::translate(0.0,-width/2, 0.0 ) *
                                  vsg::rotate(M_PI/2, vsg::dvec3(1,0,0)),
                                  conf.irRangeSide, RaySensor::drawAll);
    }
    if (conf.irBack){ // add rear right and rear left infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
        IRSensor* sensor = new IRSensor();
        irSensorBank->registerSensor(sensor, objects[0],
                                    vsg::translate(0.0,i*width/10,-(length/2 + width/2 - width/60) )*
                                    vsg::rotate(i*M_PI, vsg::dvec3(0,1,0)) *
                                    vsg::rotate(-i*M_PI/10, vsg::dvec3(1,0,0)),
                                    conf.irRangeBack, RaySensor::drawAll);
      }
    }
    if (conf.irSide){ // add left infrared sensor to sensorbank if required
        IRSensor* sensor = new IRSensor();
        irSensorBank->registerSensor(sensor, objects[0],
                                    //vsg::rotate(i*M_PI/2, vsg::dvec3(0,0,1)) *
                                    vsg::translate(0.0,width/2, 0.0) *
                                    vsg::rotate(-M_PI/2, vsg::dvec3(1,0,0)), 
                                    conf.irRangeSide, RaySensor::drawAll);
    }
    addSensor(shared_ptr<Sensor>(irSensorBank));
  };


  // returns the joint with index i
  Joint* FourWheeled::getJoint(int i){
    if(i>3)i=3;
    if(i<0)i=0;
    return joints[i];
  }


  /** destroys vehicle and space
   */
  void FourWheeled::destroy(){
    Nimm4::destroy();
  }

}

