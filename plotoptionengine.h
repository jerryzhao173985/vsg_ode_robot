#ifndef PLOTOPTIONENGINE_H_
#define PLOTOPTIONENGINE_H_

#include <list>

#include "plotoption.h"
#include "abstractcontroller.h"

class Inspectable;

/*
 *
 */
class PlotOptionEngine
{
public:
  PlotOptionEngine(const PlotOption& plotOption);
  PlotOptionEngine(const std::list<PlotOption>& plotOptions);

  virtual ~PlotOptionEngine();

  /** initializes PlotOptionEngine and opens all pipes and stuff.
      The optional controller is used to print structure information
   */
  virtual bool init(AbstractController* maybe_controller =0);

  /**
   * Reinitialises the PlotOptionEngine.
   * This means it closes all open pipes by calling closePipes()
   * and restarts them by calling init().
   * @return true if succeeded, otherwise false.
   */
  virtual bool reInit();

  /**
   * Closes all open pipes of the current used PlotOptions.
   */
  virtual void closePipes();

  /**
     sets the name of all plotoptions (call before init, but after options are added)
   */
  virtual void setName(const std::string& name);

  /** adds the PlotOptions to the list of plotoptions
      If a plotoption with the same Mode exists, then the old one is deleted first
   */
  virtual PlotOption& addPlotOption(const PlotOption& plotoption);

  /**
   * Adds the PlotOptions to the list of plotoptions
   * If a plotoption with the same Mode exists, then the old one is deleted first
   * The plotting is also initialized but only if the PlotOptionEngine is already intialized,
   * This can be forced by additional param.
   * @param plotoption The PlotOption to add
   * @param forceInit force initialization of PlotOption even if PlotOptionEngine is not initalized.
   *        This may cause problems. Use this at your own risk.
   * @return
   */
  virtual bool addAndInitPlotOption(const PlotOption& plotoption, bool forceInit = false);

  /** removes the PlotOptions with the given type
      @return true if sucessful, false otherwise
   */
  virtual bool removePlotOption(PlotMode mode);


  /** adds an inspectable object for logging. Must be called before init!
   */
  virtual void addInspectable(const Inspectable* inspectable, bool front = false);

  /** adds an configureable object for logging. Must be called before init!
   */
  virtual void addConfigurable(const Configurable* c);

  /**
     write comment to output streams (PlotOptions). For instance changes in parameters.
     If addSpace then "# CMT" is output otherwise "#CMT".
  */
  virtual void writePlotComment(const char* cmt, bool addSpace=true );

  virtual void plot(double time);

protected:

  bool initPlotOption(PlotOption& po);

  std::list<PlotOption> plotOptions;
  std::list<const Inspectable* > inspectables;
  std::list< const Configurable* > configureables;
  long int t;

  bool initialised;


  // old artefact, should be removed in future releases
  AbstractController* maybe_controller;

  std::string name; // name given to PlotOptions
};

#endif /* PLOTOPTIONENGINE_H_ */
