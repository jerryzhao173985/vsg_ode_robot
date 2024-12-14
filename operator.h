#ifndef __OPERATOR_H
#define __OPERATOR_H

#include "globaldata.h"

#include "pose.h"


namespace lpzrobots {
  /**
     An Operator observes an agent (robot) and manipulates it if necessary.
     For instance if the robot is falled over the operator can flip it back.
     This is an abstract base class and subclasses should overload at least
     observe().
   */

  class OdeAgent;

  class Operator : public Configurable {
  public:
    /** type of manipulation of the robot (for display) and or operation
        RemoveOperator means that the operator should be removed
     */
    enum ManipType {None, Limit, Move, RemoveOperator};
    /// description of action (for visualization)
    struct ManipDescr {
      ManipDescr() : show(1), size(0.05,0.05,0.05){
      }
      short show;
      Pos pos;
      Pos posStart;
      Pose orientation;
      Pos size;

    };

    Operator( const std::string& name, const std::string& revision)
      : Configurable(name, revision)
    {
    }

    virtual ~Operator(){
    }

    /** called every simulation step
        @return what was done with the robot
     */
    virtual ManipType observe(OdeAgent* agent, GlobalData& global, ManipDescr& descr) = 0;

  };

}

#endif
