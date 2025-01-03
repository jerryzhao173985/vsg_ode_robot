#ifndef __FOUR_WHEELED__
#define __FOUR_WHEELED__

#include "nimm4.h"
#include "raysensorbank.h"

namespace lpzrobots {

  class Primitive;
  class Hinge2Joint;
  class Joint;

  typedef struct {
    double size;
    double force;
    double speed;
    bool sphereWheels;
    bool useBumper;
    bool useButton; ///< use yellow Button at the back
    bool twoWheelMode; ///< if true then the robot emulates 2 wheels
    bool irFront;
    bool irBack;
    bool irSide;
    double irRangeFront;
    double irRangeBack;
    double irRangeSide;
    Substance wheelSubstance;
  } FourWheeledConf;

  /** Robot is based on nimm4 with
      4 wheels and a capsule like body
  */
  class FourWheeled : public Nimm4{
  public:

    /**
     * constructor of nimm4 robot
     * @param odeHandle data structure for accessing ODE
     * @param vsgHandle ata structure for accessing OSG
     * @param conf configuration structure
     * @param name name of the robot
     */
    FourWheeled(const OdeHandle& odeHandle, const VsgHandle& vsgHandle, FourWheeledConf conf, const std::string& name);

    static FourWheeledConf getDefaultConf(){
      FourWheeledConf conf;
      conf.size         = 1;
      conf.force        = 3;
      conf.speed        = 15;
      conf.sphereWheels = true;
      conf.twoWheelMode = false;
      conf.useBumper    = true;
      conf.useButton    = false;
      conf.irFront      = false;
      conf.irBack       = false;
      conf.irSide       = false;
      conf.irRangeFront = 3;
      conf.irRangeSide  = 2;
      conf.irRangeBack  = 2;
      conf.wheelSubstance.toRubber(40);
      return conf;
    }

    virtual ~FourWheeled();

    virtual int getSensorNumberIntern();
    virtual int getMotorNumberIntern();

    virtual int getSensorsIntern(sensor* sensors, int sensornumber);

    virtual void setMotorsIntern(const double* motors, int motornumber);

    // returns the joint with index i
    virtual Joint* getJoint(int i);

  protected:
    /** creates vehicle at desired pose
        @param pose 4x4 pose matrix
    */
    virtual void create(const vsg::dmat4& pose);

    /** destroys vehicle and space
     */
    virtual void destroy();

    FourWheeledConf conf;
    Primitive* bumpertrans;
    Primitive* bumper;
  };

}

#endif
