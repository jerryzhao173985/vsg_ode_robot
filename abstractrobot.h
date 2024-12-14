#ifndef __ABSTRACTROBOT_H
#define __ABSTRACTROBOT_H

#include <vector>
#include <list>
#include <string>

#include "trackable.h"
#include "configurable.h"
#include "position.h"
#include "sensormotorinfo.h"

/**
 * Abstract class (interface) for robot in general
 *
 *
 */
class AbstractRobot : public Trackable, public Configurable {
public:
  typedef double sensor;
  typedef double motor;

  /**
   * Constructor
   * @param name name of the robot
   * @param revision revision number of the file (Hint: use CVS variable \verbatim $ID$ \endverbatim )
   */
  AbstractRobot(const std::string& name="abstractRobot", const std::string& revision = "$ID$")
    : Configurable(name, revision) {
  };

  virtual ~AbstractRobot(){}

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1]
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  virtual int getSensors(sensor* sensors, int sensornumber)=0;

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  virtual void setMotors(const motor* motors, int motornumber)=0;

  /** returns number of sensors
  */
  virtual int getSensorNumber()=0;

  /** returns number of motors
  */
  virtual int getMotorNumber()=0;

  virtual std::string getTrackableName() const {return getName();}

  /** returns the information for the sensors.
      The following relation has to hold: getSensorNames().size() == getSensorNumber()
   */
  virtual std::list<SensorMotorInfo> getSensorInfos()
  { return std::list<SensorMotorInfo>(getSensorNumber());};

  /** returns the information for the motors.
      The following relation has to hold: getMotorNames().size() == getMotorNumber()
   */
  virtual std::list<SensorMotorInfo> getMotorInfos()
  { return std::list<SensorMotorInfo>(getMotorNumber());};


};

#endif

