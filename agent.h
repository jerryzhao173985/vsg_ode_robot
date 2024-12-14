#ifndef __AGENT_H
#define __AGENT_H

#include <stdio.h>
#include <list>
#include <string>

#include "wiredcontroller.h"
#include "randomgenerator.h"

class AbstractRobot;

#include "types.h"
#include "trackrobots.h"

/** The Agent contains a controller, a robot and a wiring, which connects robot and controller.
    Additionally there are some ways to keep track of internal information.
    You have the possibility to keep track of sensor values,
     motor values and internal parameters of the controller with PlotOptions.
    The name PlotOptions is a bit missleaded, it should be "OutputOptions",
     however you can write the data into a file or send it to visialisation tools like
     guilogger or neuronviz.

    If want to log the position, speed and orienation of your robot
    you can use setTrackOptions().
    Please be aware that the Agent inherits from WiredController. You
     might also find useful functions there.
 */
class Agent : public WiredController {
public:
  /** constructor. PlotOption as output setting.
      noisefactor is used to set the relative noise strength of this agent
   */
  Agent(const PlotOption& plotOption = PlotOption(NoPlot), double noisefactor = 1, const iparamkey& name = "Agent", const paramkey& revision = "$ID");
  /** constructor. A list of PlotOption can given.
      noisefactor is used to set the relative noise strength of this agent
   */
  Agent(const std::list<PlotOption>& plotOptions, double noisefactor = 1, const iparamkey& name = "Agent", const paramkey& revision = "$ID");

  /** destructor
   */
  virtual ~Agent();

  /** initializes the object with the given controller, robot and wiring
      and initializes the output options.
      It is also possible to provide a random seed,
       if not given (0) rand() is used to create one
  */
  virtual bool init(AbstractController* controller, AbstractRobot* robot,
                    AbstractWiring* wiring, long int seed=0);

  /** Performs an step of the agent, including sensor reading, pushing sensor values through the wiring,
      controller step, pushing controller outputs (= motorcommands) back through the wiring and sent
      resulting motorcommands to robot.
      @param noise Noise strength.
      @param time (optional) current simulation time (used for logging)
  */
  virtual void step(double noise, double time=-1);

  /** Sends only last motor commands again to robot.  */
  virtual void onlyControlRobot();


  /** Returns a pointer to the robot.
   */
  virtual AbstractRobot* getRobot() { return robot; }

  /// sets the trackoptions which starts spatial tracking of a robot
  virtual void setTrackOptions(const TrackRobot& trackrobot);

  /// stop tracking (returns true of tracking was on);
  virtual bool stopTracking();

  /// returns the tracking options
  virtual TrackRobot getTrackOptions() const { return trackrobot; }

protected:

  AbstractRobot* robot;

  sensor *rsensors;
  motor  *rmotors;

  RandGen randGen; // random generator for this agent

  TrackRobot trackrobot;
  int t; // access to this variable is needed from OdeAgent


};

#endif
