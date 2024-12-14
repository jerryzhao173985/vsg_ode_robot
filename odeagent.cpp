#include "odeagent.h"
#include "oderobot.h"
#include "globaldata.h"
#include "joint.h"
#include "pos.h"
#include "tmpprimitive.h"
#include <assert.h>
#include "matrix.h"
#include <vsg/all.h>

// rotation and translation matrixes (to make the code shorter)
// #define vsg::rotate vsg::rotate
// #define vsg::translate vsg::translate

namespace lpzrobots {

  /**
   * This function calculates how to smoothly move and rotate between two positions in 3D space. 
   * It takes two positions (a starting and ending point) as input, figures out which direction it needs to move by subtracting the positions,
   * calculates how much rotation is needed to turn from straight up (the z-axis) to point in that direction, and creates a transformation matrix that combines:
   * The rotation to point in the right direction
   * Moving to the midpoint between the start and end positions
   */
  vsg::dmat4 poseToPose(const Position& lastpos, const Position& pos) {
    // First convert Position to Pos for calculations
    Position diff = pos - lastpos;  // This works because Position has operator-
    Pos direction(diff.x, diff.y, diff.z);  // Convert Position to Pos

    // Get the z-axis vector we're rotating from
    Pos zAxis(0.0, 0.0, 1.0);

    // Calculate the angle between vectors using dot product
    double dot = (direction * zAxis);  // Pos has operator* for dot product
    double dirLength = diff.length();
    if (dirLength > 0.0) 
    {
        double angle = std::acos(dot / dirLength);  // angle between vectors
        
        // Calculate rotation axis using cross product
        Pos rotationAxis = Pos(
            zAxis.y() * direction.z() - zAxis.z() * direction.y(),
            zAxis.z() * direction.x() - zAxis.x() * direction.z(),
            zAxis.x() * direction.y() - zAxis.y() * direction.x()
        );
        
        // Translation to midpoint
        Position midpoint = lastpos + (diff * 0.5);  // Using Position operators
        
        return vsg::rotate(angle, rotationAxis) * 
              vsg::translate(Pos(midpoint.x, midpoint.y, midpoint.z));
    }

    return vsg::translate(Pos(lastpos.x, lastpos.y, lastpos.z));
  }

  

  void TraceDrawer::init(){
    assert(obj);
    lastpos = obj->getPosition();
    initialized=true;
  }

  void TraceDrawer::close(){
    if(initialized)
      tracker.close();
    initialized=false;
  }

  void TraceDrawer::track(double time){
    if (initialized){
      tracker.track(obj, time);
    }
  }

  void TraceDrawer::drawTrace(GlobalData& global){
    if (initialized && tracker.isDisplayTrace()){
      Position pos(obj->getPosition());
      double len = (pos - lastpos).length();
      if(tracker.conf.displayTraceThickness>0){ // use a cylinder
        /* draw cylinder only when length between actual
           and last point is larger then a specific value
        */
        if(len > 2*tracker.conf.displayTraceThickness) {
          auto cylinder = new Cylinder(tracker.conf.displayTraceThickness, len*1.2);
          cylinder->init(global.odeConfig.odeHandle, 0.0, global.vsgHandle, Primitive::Geom | Primitive::Draw);
          global.addTmpObject(new TmpDisplayItem(cylinder,
                                                 poseToPose(lastpos, pos),
                                                 color),
                              tracker.conf.displayTraceDur);
          lastpos = pos;
        }
      }else{ // use a line
        if(len > 0.05) {
          // pnts.push_back(Pos(lastpos));
          // pnts.push_back(Pos(pos));
          // if(pnts.size()>16){
          //   global.addTmpObject(new TmpDisplayItem(new /*Line*/Box(pnts), vsg::translate(0,0,0), color),
          //                       tracker.conf.displayTraceDur);
          //   pnts.clear();
          // }
          lastpos = pos;
        }
      }
    }
  }


  OdeAgent::OdeAgent(const PlotOption& plotOption, double noisefactor, const std::string& name, const std::string& revision)
    : Agent(plotOption, noisefactor, name, revision) {
    constructor_helper(0);
  }
  OdeAgent::OdeAgent(const std::list<PlotOption>& plotOptions, double noisefactor, const std::string& name, const std::string& revision)
    : Agent(plotOptions, noisefactor, name, revision) {
    constructor_helper(0);
  }

  OdeAgent::OdeAgent(const GlobalData& globalData, double noisefactor, const std::string& name, const std::string& revision)
    : Agent(globalData.plotoptions, noisefactor, name, revision){
    constructor_helper(&globalData);
  }

  OdeAgent::OdeAgent(const GlobalData& globalData, const PlotOption& plotOption,
                     double noisefactor, const std::string& name, const std::string& revision)
    : Agent(plotOption, noisefactor, name, revision){
    constructor_helper(&globalData);
  }


  OdeAgent::OdeAgent(const GlobalData& globalData, const PlotOptionList& plotOptions,
                     double noisefactor, const std::string& name, const std::string& revision)
    : Agent(plotOptions, noisefactor, name, revision){
    constructor_helper(&globalData);
  }

  OdeAgent::~OdeAgent(){
    removeOperators();
    FOREACH(TraceDrawerList, segmentTracking, td){
      td->close();
    }

  }

  void OdeAgent::constructor_helper(const GlobalData* globalData){
    if(globalData){
      FOREACHC(std::list<Configurable*>, globalData->globalconfigurables, c){
        plotEngine.addConfigurable(*c);
      }
    }
  }

  void OdeAgent::step(double noise, double time){
    Agent::step(noise, time);
    // for the main trace we do not call track, this in done in agent
    // track the segments
    FOREACH(TraceDrawerList, segmentTracking, td){
      td->track(time);
    }
  }

  void OdeAgent::beforeStep(GlobalData& global){
    OdeRobot* r = getRobot();
    r->sense(global);

    trace(global);

    Operator::ManipDescr d;
    Operator::ManipType m;
    FOREACH(OperatorList, operators, i){
      m=(*i)->observe(this, global, d);
      switch(m){
      case Operator::RemoveOperator:
        delete *i;
        i=operators.erase(i);
        if(i!=operators.end()) i--;
        break;
      case Operator::Move:
        if(d.show){
          global.addTmpObject(new TmpDisplayItem(new Sphere(d.size.x()),
                                                 vsg::translate(d.pos), "manipmove"),
                              global.odeConfig.simStepSize*5);
          // if(d.show>1)
          //   global.addTmpObject(new TmpDisplayItem(new Line({d.posStart,d.pos}),
          //                                          vsg::translate(0,0,0), "manipmove"),
          //                       global.odeConfig.simStepSize
          //                       );
        }
        break;
      case Operator::Limit:
        if(d.show){
          global.addTmpObject(new TmpDisplayItem(new Cylinder(d.size.x(),d.size.z()),
                                                 d.orientation * vsg::translate(d.pos),
                                                 "maniplimit", 0.2),0.5);
        }
        break;
      default: break;
      }
    }
  }

  void OdeAgent::stepOnlyWiredController(double noise, double time) {
    WiredController::step(rsensors,rsensornumber, rmotors, rmotornumber, noise, time);
    trackrobot.track(robot, time); // we have to do this here because agent.step is not called
    // track the segments
    FOREACH(TraceDrawerList, segmentTracking, td){
      td->track(time);
    }

  }

  void OdeAgent::trace(GlobalData& global){
    mainTrace.drawTrace(global);
    FOREACH(TraceDrawerList, segmentTracking, td){
      td->drawTrace(global);
    }
  }

  void OdeAgent::setTrackOptions(const TrackRobot& trackrobot){
    std::cout << "OdeAgent::setTrackOptions()" << std::endl;
    Agent::setTrackOptions(trackrobot);
    if (trackrobot.isDisplayTrace()){
      mainTrace.obj=robot;
      mainTrace.tracker = trackrobot;
      mainTrace.color = ((OdeRobot*)robot)->vsgHandle.color;
      mainTrace.init();
    }
  }

  bool OdeAgent::stopTracking(){
    bool rv=Agent::stopTracking();
    // we also want to stop the trace:
    mainTrace.close();

    return rv;

  }

  class TrackablePrimitive : public Trackable {
  public:
    TrackablePrimitive(Primitive* p, const std::string& name)
      : p(p), name(name) { }
    virtual std::string getTrackableName() const { return name; };
    virtual Position getPosition() const  { return p->getPosition().toPosition(); };
    virtual Position getSpeed() const     { return p->getVel().toPosition(); };
    virtual Position getAngularSpeed() const { return p->getAngularVel().toPosition(); };
    virtual matrix::Matrix getOrientation() const {
      fprintf(stderr, "TrackablePrimitive:: getOrientation(): not implemented\n");
      return matrix::Matrix(3,3);
    };

  protected:
    Primitive* p;
    std::string name;
  };

  /// adds tracking for individual primitives
  void OdeAgent::addTracking(unsigned int primitiveIndex,const TrackRobot& trackrobot,
                             const Color& color){
    assert(robot);
    TraceDrawer td;
    Primitives ps = ((OdeRobot*)robot)->getAllPrimitives();
    if(primitiveIndex >= ps.size()){
      fprintf(stderr, "OdeAgent::addTracking(): primitive index out of bounds %ui", primitiveIndex);
      return;
    }
    td.obj=new TrackablePrimitive(ps[primitiveIndex],
                                  ((OdeRobot*)robot)->getName() + "segm_" + std::to_string(primitiveIndex));
    td.tracker = trackrobot;
    td.tracker.conf.id=primitiveIndex;
    td.color = color;
    td.init();
    if(!td.tracker.open(robot)){
      fprintf(stderr, "OdeAgent.cpp() ERROR: could not open trackfile! <<<<<<<<<<<<<\n");
    }
    segmentTracking.push_back(td);
  }


  void OdeAgent::setMotorsGetSensors() {
    robot->setMotors(rmotors, rmotornumber);

    assert(robot && rsensors && rmotors);

    int len =  robot->getSensors(rsensors, rsensornumber);
    if(len != rsensornumber){
      fprintf(stderr, "%s:%i: Got not enough sensors, expected %i, got %i!\n", __FILE__, __LINE__,
        rsensornumber, len);
    }
  }

  void OdeAgent::fixateRobot(GlobalData& global, int primitiveID, double time){
    OdeRobot* r = dynamic_cast<OdeRobot*>(robot);
    if(!r) return;
    r->fixate(global,primitiveID,time);
  }

  bool OdeAgent::unfixateRobot(GlobalData& global){
    OdeRobot* r = dynamic_cast<OdeRobot*>(robot);
    if(!r) return false;
    return r->unFixate(global);
  }


  bool OdeAgent::store(FILE* f) const {
    const OdeRobot* r = getRobot();
    return r->store(f) && getController()->store(f);
  }

  bool OdeAgent::restore(FILE* f){
    OdeRobot* r = getRobot();
    return r->restore(f) && getController()->restore(f);
  }



  void OdeAgent::addOperator(Operator* o, bool addToConfigurable){
    if(o){
      operators.push_back(o);
      if(addToConfigurable){
        addConfigurable(o);
      }
    }

  }

  bool OdeAgent::removeOperator(Operator* o){
    unsigned int size = operators.size();
    operators.remove(o);
    removeConfigurable(o);
    return operators.size() < size;
  }

  void OdeAgent::removeOperators(){
    FOREACHC(OperatorList, operators, i){
      removeConfigurable(*i);
      delete (*i);
    }
    operators.clear();
  }


}
