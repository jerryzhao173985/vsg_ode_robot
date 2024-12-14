#ifndef __MOTORBABBLER_H
#define __MOTORBABBLER_H

#include <stdio.h>
#include "abstractcontroller.h"
#include "matrix.h"

/**
 * class for robot control that does motor babbling, e.g. sine waves
 * with different frequencies and phaseshift
 */
class MotorBabbler : public AbstractController {
public:
  enum function {Sine, SawTooth};

  /**
     @param controlmask bitmask to select channels to control (default all)
     @param function controller function to use
   */
  MotorBabbler(function func = Sine );

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);
  virtual int getSensorNumber() const {return number_sensors;}
  virtual int getMotorNumber() const {return number_motors;}
  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber){
    stepNoLearning(sensors, sensornumber, motors, motornumber);
  }
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  // samples a new set of frequencies
  void sampleFrequencies();

  /********* STORABLE INTERFACE ******/
  /// @see Storable
  virtual bool store(FILE* f) const {
    Configurable::print(f,"");
    return true;
  }

  /// @see Storable
  virtual bool restore(FILE* f) {
    Configurable::parse(f);
    return true;
  }

  /// sine
  static double sine(double x);
  /// saw tooth shape oscillator
  static double sawtooth(double x);

protected:
  int number_sensors;
  int number_motors;

  paramval minPeriod;
  paramval maxPeriod;
  paramval amplitude;
  paramint resampling;
  paramint mask;
  matrix::Matrix phases;
  matrix::Matrix frequencies;

  RandGen* randGen;

  double (*osci) (double x); // oscillator function
  int t;
};

#endif
