#ifndef __ONE2ONEWIRING_H
#define __ONE2ONEWIRING_H

#include "abstractwiring.h"

/** Implements one to one wiring of robot sensors to inputs of the controller
    and controller outputs to robot motors.
 */
class One2OneWiring :public AbstractWiring{
public:
  /** constructor
      @param noise NoiseGenerator that is used for adding noise to sensor values
      @param plotMode see AbstractWiring
      @param blind number of blind channels
        (additional sensors and motors coupled directly)
   */
  One2OneWiring(NoiseGenerator* noise, int plotMode=Controller, int blind=0, const std::string& name = "One2OneWiring");

  /** destructor
   */
  virtual ~One2OneWiring();

protected:

  /** initializes the number of sensors and motors on robot side, calculate
      number of sensors and motors on controller side
   */
  virtual bool initIntern();

  /** Realizes one to one wiring from robot sensors to controller sensors.
      @param rsensors pointer to array of sensorvalues from robot
      @param rsensornumber number of sensors from robot
      @param csensors pointer to array of sensorvalues for controller
      @param csensornumber number of sensors to controller
      @param noise size of the noise added to the sensors
  */
  virtual bool wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                           sensor* csensors, int csensornumber,
                           double noise);

  /** Realizes one to one wiring from controller motor outputs to robot motors.
      @param rmotors pointer to array of motorvalues for robot
      @param rmotornumber number of robot motors
      @param cmotors pointer to array of motorvalues from controller
      @param cmotornumber number of motorvalues from controller
  */
  virtual bool wireMotorsIntern(motor* rmotors, int rmotornumber,
                          const motor* cmotors, int cmotornumber);


protected:
  int blind; /// number of blind channels
  /// blind motor values
  motor* blindmotors;

};

#endif
