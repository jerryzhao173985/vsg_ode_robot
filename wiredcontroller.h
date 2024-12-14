#ifndef     WIREDCONTROLLER_H_
#define     WIREDCONTROLLER_H_

#include "plotoptionengine.h"
#include "backcaller.h"
#include "types.h"
#include "inspectable.h"
#include "configurable.h"
#include "randomgenerator.h"

#include <stdio.h>
#include <list>
#include <utility>
#include <string>


class AbstractController;
class AbstractWiring;
class Callbackable;

/** The WiredController contains a controller and a wiring, which
    connects the controller with the robot.
    Additionally there are some ways to keep track of internal information.
    You have the possibility to keep track of sensor values,
     motor values and internal parameters of the controller with PlotOptions.
    The name PlotOptions is a bit missleaded, it should be "OutputOptions",
     however you can write the data into a file or send it to
     visualisation tools like guilogger or neuronviz.
 */
class WiredController : public Inspectable, public Configurable {
public:
  /** constructor. PlotOption as output setting.
      noisefactor is used to set the relative noise strength of this agent
   */
  WiredController(const PlotOption& plotOption = PlotOption(NoPlot), double noisefactor = 1, const iparamkey& name = "WiredController", const paramkey& revision = "$ID");
  /** constructor. A list of PlotOption can given.
      noisefactor is used to set the relative noise strength of this agent
   */
  WiredController(const std::list<PlotOption>& plotOptions, double noisefactor = 1, const iparamkey& name = "WiredController", const paramkey& revision = "$ID");

  /** destructor
   */
  virtual ~WiredController();

  /** initializes the object with the given controller and wiring
      and initializes the output options
      It is also possible to provide a random seed,
       if not given (0) rand() is used to create one
  */
  virtual bool init(AbstractController* controller, AbstractWiring* wiring,
                    int robotsensornumber, int robotmotornumber,
                    const std::list<SensorMotorInfo>& robotSensorInfos,
                    const std::list<SensorMotorInfo>& robotMotorInfos,
                    RandGen* randGen=0);

  /** Performs an step of the controller, which includes
      pushing sensor values through the wiring,
      controller step,
      pushing controller outputs (= motorcommands) back through the wiring
      @param sensors sensors inputs scaled to [-1,1]
      @param sensornumber length of the sensor array
      @param motors motors outputs. MUST have enough space for motor values!
      @param motornumber length of the provided motor array

      @param noise Noise strength.
      @param time (optional) current simulation time (used for logging)
  */
  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber,
                    double noise, double time=-1);

  /** Enables the motor babbling mode for given number of steps (typically 1000).
      Optionally a controller can be
      given that is used for the babbling (default is MotorBabbler) (deleted automatically).
      During motor babbling the function motorbabbling of the normal controller is called instead of step.
   */
  virtual void startMotorBabblingMode (int steps, AbstractController* babblecontroller = 0);

  virtual AbstractController* getMotorBabbler() { return motorBabbler; }

  /** stops the motor babbling mode. */
  virtual void stopMotorBabblingMode () { motorBabblingSteps = 0; }
  /// returns true if in motorbabbling mode
  virtual bool getMotorBabblingMode()  { return motorBabblingSteps > 0; }


  /** adds the PlotOptions to the list of plotoptions
      If a plotoption with the same Mode exists, then the old one is deleted first
   */
  virtual PlotOption addPlotOption(PlotOption& plotoption);

  /** adds a new PlotOption and initializes it
      @see addPlotOption
  */
  virtual bool addAndInitPlotOption(PlotOption& plotOption);

  /** removes the PlotOptions with the given type
      @return true if sucessful, false otherwise
   */
  virtual bool removePlotOption(PlotMode mode);

  /**
     write comment to output streams (PlotOptions). For instance changes in parameters.
     see PlotOptionEngine
  */
  virtual void writePlotComment(const char* cmt, bool addSpace=true);

  /** Returns a pointer to the controller.
   */
  virtual AbstractController* getController() { return controller;}
  virtual const AbstractController* getController() const { return controller;}

  /** Returns a pointer to the wiring.
   */
  virtual AbstractWiring* getWiring() { return wiring;}

protected:
  /**
   * Plots controller sensor- and motorvalues and internal controller parameters.
   * @param time simulation time
   */
  virtual void plot(double time);


  AbstractController* controller;
  AbstractWiring* wiring;

  /// number of sensors of robot
  int rsensornumber;
  /// number of motors of robot
  int rmotornumber;
  /// number of sensors of comntroller
  int csensornumber;
  /// number of motors of comntroller
  int cmotornumber;

  /// factor that is  muliplied with noise stength
  double noisefactor;

  motor  *cmotors;
  sensor *csensors;

  void internInit();

 protected:
  AbstractController* motorBabbler;
  int motorBabblingSteps;

  PlotOptionEngine plotEngine;

  bool initialised;

  std::list<Callbackable* > callbackables;

  long int t;
};

#endif       /* !WIREDCONTROLLER_H_ */
