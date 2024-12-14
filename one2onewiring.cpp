#include "one2onewiring.h"
#include <assert.h>
#include <cstring>

/// constructor
One2OneWiring::One2OneWiring(NoiseGenerator* noise, int plotMode, int blind, const std::string& name)
  : AbstractWiring(noise, plotMode, name), blind(blind){
  blindmotors=0;
}

One2OneWiring::~One2OneWiring(){
  if(blindmotors) free(blindmotors);
}


/// initializes the number of sensors and motors from robot, calculate
//  number of sensors and motors on controller side
bool One2OneWiring::initIntern(){
  csensornumber = rsensornumber+blind;
  cmotornumber  = rmotornumber+blind;
  noisenumber   = csensornumber;

  if(blind){
    blindmotors = (sensor*) malloc(sizeof(sensor)  * blind);
    memset(blindmotors, 0, sizeof(sensor)  * blind);
  }

  return true;
}

/// Realizes one to one wiring from robot sensors to controller sensors.
//   @param rsensors pointer to array of sensorvalues from robot
//   @param rsensornumber number of sensors from robot
//   @param csensors pointer to array of sensorvalues for controller
//   @param csensornumber number of sensors to controller
//   @param noise size of the noise added to the sensors
bool One2OneWiring::wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                                      sensor* csensors, int csensornumber,
                                      double noiseStrength){
  assert(rsensornumber == this->rsensornumber);
  assert(csensornumber == this->csensornumber);
  // the noisevals are set in abstractwiring
  for(int i=0; i< rsensornumber; i++){
    csensors[i] = rsensors[i] + noisevals[i];
  }
  for(int i=0; i< blind; i++){
    csensors[i + rsensornumber] = blindmotors[i] + noisevals[rsensornumber+i];
  }
  return true;
}


/// Realizes one to one wiring from controller motor outputs to robot motors.
//   @param rmotors pointer to array of motorvalues for robot
//   @param rmotornumber number of robot motors
//   @param cmotors pointer to array of motorvalues from controller
//   @param cmotornumber number of motorvalues from controller
bool One2OneWiring::wireMotorsIntern(motor* rmotors, int rmotornumber,
                                     const motor* cmotors, int cmotornumber){
  assert(rmotornumber == this->rmotornumber);
  assert(cmotornumber == this->cmotornumber);
  memcpy(rmotors, cmotors, sizeof(motor)*rmotornumber);
  if(blind)
    memcpy(blindmotors, cmotors+rmotornumber, sizeof(motor)*blind);
  return true;
}


