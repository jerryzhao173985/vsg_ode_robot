#ifndef __TEACHABLE_H
#define __TEACHABLE_H

#include "matrix.h"

/**
   Interface for teachable controller. 
*/
class Teachable {
public:
  
  virtual ~Teachable() {}

  /** The given motor teaching signal is used for this timestep. 
      It is used as a feed forward teaching signal for the controller.
      Please note, that the teaching signal has to be given each timestep 
       for a continuous teaching process.
     @param teaching: matrix with dimensions (motornumber,1)
   */
  virtual void setMotorTeaching(const matrix::Matrix& teaching) = 0;

  /** The given sensor teaching signal (distal learning) is used for this timestep. 
      The belonging motor teachung signal is calculated by the inverse model.
      See setMotorTeaching
     @param teaching: matrix with dimensions (motorsensors,1)
   */
  virtual void setSensorTeaching(const matrix::Matrix& teaching) = 0;

  /// returns the last motor values (useful for cross motor coupling)
  virtual matrix::Matrix getLastMotorValues() = 0;

  /// returns the last sensor values (useful for cross sensor coupling)
  virtual matrix::Matrix getLastSensorValues() = 0;
};

  
#endif
