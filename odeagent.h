#ifndef __ODEAGENT_H
#define __ODEAGENT_H

#include "agent.h"
#include "storeable.h"
#include "oderobot.h"
#include "primitive.h"
#include "operator.h"

namespace lpzrobots {
  class Joint;

  class TraceDrawer {
  public:
    TraceDrawer() : obj(0), initialized(false) {}
    Position lastpos;
    Trackable* obj;
    TrackRobot tracker;
    Color color;
    void init();
    void close();
    /// actually write the log files and stuff
    void track(double time);
    /// draw the trace
    void drawTrace(GlobalData& global);
  protected:
    bool initialized;
    // std::list<vsg::dvec3> pnts;
  };


  typedef std::list<PlotOption> PlotOptionList;
  typedef std::list<Operator*> OperatorList;
  typedef std::list<TraceDrawer> TraceDrawerList;

  /** Specialised agent for ode robots
   */
  class OdeAgent : public Agent, public Storeable {
  public:

    /** @deprecated obsolete provide globaldata, see the other constructors
     */
    OdeAgent(const PlotOption& plotOption = PlotOption(NoPlot), double noisefactor = 1, const std::string& name = "OdeAgent", const std::string& revision = "$ID$") __attribute__ ((deprecated));
    /** @deprecated obsolete provide globaldata, see the other constructors
     */
    OdeAgent(const std::list<PlotOption>& plotOptions, double noisefactor = 1, const std::string& name = "OdeAgent", const std::string& revision = "$ID$") __attribute__ ((deprecated));
    /** The plotoptions are taken from globaldata
        @param noisefactor factor for sensor noise for this agent
     */
    OdeAgent(const GlobalData& globalData, double noisefactor = 1, const std::string& name = "OdeAgent", const std::string& revision = "");
    /** Provided for convinience. A single plotoption is used as given by plotOption */
    OdeAgent(const GlobalData& globalData, const PlotOption& plotOption, double noisefactor = 1, const std::string& name = "OdeAgent", const std::string& revision = "");
    /** Provided for convinience. The plotoptions are taken from the given plotOptions
        (and not from globaldata, if you wish to overwrite them)
    */
    OdeAgent(const GlobalData& globalData, const PlotOptionList& plotOptions, double noisefactor = 1, const std::string& name = "OdeAgent", const std::string& revision = "");
    virtual ~OdeAgent();

    /** initializes the object with the given controller, robot and wiring
        and initializes plotoptionengine
    */
    virtual bool init(AbstractController* controller, OdeRobot* robot, AbstractWiring* wiring,
                      long int seed = 0){
      return Agent::init(controller, robot, wiring, seed);
    }

    virtual void step(double noise, double time);

    /**
     * Special function for the class Simulation to seperate the step
     * of the WiredController (plus TrackRobot) and the setting and getting
     * of the motor- and sensorvalues.
     * @param noise @see step()
     * @param time @see step()
     */
    virtual void stepOnlyWiredController(double noise, double time);

    /**
     * Special function for the class Simulation to seperate the step
     * of the WiredController (plus TrackRobot) and the setting and getting
     * of the motor- and sensorvalues.
     */
    virtual void setMotorsGetSensors();

    /** should be called before step() or stepOnlyWiredController()
        and calls operators and robot->sense()
    */
    virtual void beforeStep(GlobalData& global);

    /**
     * Returns a pointer to the robot.
     */
    virtual OdeRobot* getRobot() { return (OdeRobot*)robot;}
    /**
     * Returns a const pointer to the robot.
     */
    virtual const OdeRobot* getRobot() const { return (OdeRobot*)robot;}

    /** @deprecated use TrackRobot parameters */
    virtual int getTraceLength(){return 0;}

    /** @deprecated use TrackRobot parameters */
    virtual bool setTraceLength(int tracelength) {return true;}

    /** @deprecated use TrackRobot parameters */
    virtual void setTraceThickness(int tracethickness){ }

    /// adds tracking for individual primitives
    virtual void addTracking(unsigned int primitiveIndex,const TrackRobot& trackrobot,
                             const Color& color);
    virtual void setTrackOptions(const TrackRobot& trackrobot);
    virtual bool stopTracking();


    /****** STOREABLE **********/
    virtual bool store(FILE* f) const;
    virtual bool restore(FILE* f);


    /****** OPERATORS *********/
    /// adds an operator to the agent (the operator is deleted on destruction of the agent!)
    virtual void addOperator(Operator* o, bool addToConfigurable = true );

    /** removes the given operator: it is _not_ deleted (memory wise)
        @return true on success
     */
    virtual bool removeOperator(Operator* o);
    /// removes (and deletes) all operators
    virtual void removeOperators();

    /** fixates the given primitive of the robot at its current position to the world
        for a certain time.
        Hint: use getRobot()->moveToPosition() to get the robot relocated
        @param primitiveID if -1 then the main primitive is used, otherwise the primitive with the given index
        @param time time to fixate in seconds (if ==0 then indefinite)
     */
    virtual void fixateRobot(GlobalData& global, int primitiveID=-1, double time = 0);
    /// release the robot in case it is fixated and turns true in this case
    virtual bool unfixateRobot(GlobalData& global);

  protected:

    /**
     * continues the trace by one segment
     */
    virtual void trace(GlobalData& global);

  private:
    void constructor_helper(const GlobalData* globalData);

    TraceDrawer mainTrace;

    OperatorList operators;

    TraceDrawerList segmentTracking;
  };

}

#endif
