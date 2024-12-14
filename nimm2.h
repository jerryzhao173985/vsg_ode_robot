#ifndef __NIMM2_H
#define __NIMM2_H

#include "oderobot.h"
#include "raysensorbank.h"

#include "primitive.h"
#include "joint.h"
#include "inspectable.h"


namespace lpzrobots {

typedef struct Bumper{
  Bumper() { trans = 0; bump = 0;}
  Primitive* trans;
  Primitive* bump;
} Bumper;

typedef struct {
  double size;
  double force;
  double speed;
  double massFactor;
  bool sphereWheels;
  double wheelSize; ///< size of the wheels in body diameters
  double wheelSlip;
  double wheelOffset; ///< offset of the wheel position in body length (not size) (<0 automatic)
  std::string wheelTexture;
  bool bumper;
  bool cigarMode;
  double cigarLength;
  bool irFront;
  bool irBack;
  bool irSide;
  double irRange;
  bool singleMotor;
  bool visForce;
  bool boxMode;
  double  boxWidth;
} Nimm2Conf;

/** Robot that looks like a Nimm 2 Bonbon :-)
    2 wheels and a cylinder like body

    wheel order: left, right
    IR order: front left, front right, right, right rear,
              rear right, rear left, left rear, left
*/
class Nimm2 : public OdeRobot /*, public Inspectable*/ {
public:

  Nimm2(const OdeHandle& odehandle, const VsgHandle& vsgHandle,
        const Nimm2Conf& conf, const std::string& name);

  static Nimm2Conf getDefaultConf(){
    Nimm2Conf conf;
    conf.size=1;
    conf.force=5;
    conf.speed=12;
    conf.massFactor=1;
    conf.sphereWheels=true;
    conf.wheelSize=1;
    conf.wheelOffset=-1.0; // disabled
    conf.wheelSlip=0;
    conf.bumper=false;
    conf.cigarMode=false;
    conf.cigarLength=2.0;
    conf.irFront=false;
    conf.irBack=false;
    conf.irSide=false;
    conf.irRange=3;
    conf.singleMotor=false;
    conf.visForce=false;
    conf.boxMode=false;
    conf.boxWidth=1.0;
    conf.wheelTexture="Images/tire.rgb";
    return conf;
  }

  virtual ~Nimm2();

  /**
   * updates the VSG nodes of the vehicle
   */
  virtual void update();

  /** sets the pose of the vehicle
      @param pose desired 4x4 pose matrix
  */
  virtual void placeIntern(const vsg::dmat4& pose);

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1]
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  virtual int getSensorsIntern(sensor* sensors, int sensornumber);

  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  virtual void setMotorsIntern(const double* motors, int motornumber);

  /** returns number of sensors
   */
  virtual int getSensorNumberIntern(){
    return sensorno;
  };

  /** returns number of motors
   */
  virtual int getMotorNumberIntern(){
    return motorno;
  };

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments)
      @return length of the list
  */
  virtual int getSegmentsPosition(std::vector<Position> &poslist);


    /** this function is called in each timestep. It should perform robot-internal checks,
      like space-internal collision detection, sensor resets/update etc.
      @param globalData structure that contains global data from the simulation environment
   */
  virtual void doInternalStuff(GlobalData& globalData);

        virtual double& getSumForce() { return sumForce; }

        virtual double& getContactPoints() { return contactPoints; }

protected:

  double contactPoints;


  /** creates vehicle at desired pose
      @param pose 4x4 pose matrix
  */
  virtual void create(const vsg::dmat4& pose);



  /** destroys vehicle and space
   */
  virtual void destroy();
  static void mycallback(void *data, dGeomID o1, dGeomID o2);

        /**
         * Inspectable interface
         */
        /*
        virtual std::list<iparamkey> getInternalParamNames() const  { return std::list<iparamkey>(); }

        virtual std::list<iparamval> getInternalParams() const { return std::list<iparamval>(); }*/
        /*
        virtual std::list<Inspectable::iparamkey> getInternalParamNames() const;

        virtual std::list<Inspectable::iparamval> getInternalParams() const;
        */

  Nimm2Conf conf;

  double length;  // chassis length
  double width;  // chassis width
  double height;   // chassis height
  double radius;  // wheel radius
  double wheelthickness; // thickness of the wheels
  double cmass;    // chassis mass
  double wmass;    // wheel mass
  int sensorno;      //number of sensors
  int motorno;       // number of motors

  bool created;      // true if robot was created
  double max_force;

  double  wheeloffset; // offset from center when in cigarMode
  int number_bumpers;  // number of bumpers (1 -> bumpers at one side, 2 -> bumpers at 2 sides)
  Bumper bumper[2];

  bool visForce; // decides if contact force is made visible in guilogger
  double sumForce; // stores the contact force made by collisions with external objects

};

}

#endif
