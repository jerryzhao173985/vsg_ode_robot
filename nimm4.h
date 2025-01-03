#ifndef __NIMM4_H
#define __NIMM4_H

#include "oderobot.h"

namespace lpzrobots {

  class Primitive;
  class Hinge2Joint;

  /** Robot that looks like a Nimm 2 Bonbon :-)
      4 wheels and a capsule like body
      Wheelorder: front left, front right, rear left, rear right

  */
  class Nimm4 : public OdeRobot{
  public:

    /**
     * constructor of nimm4 robot
     * @param odeHandle data structure for accessing ODE
     * @param vsgHandle ata structure for accessing VSG
     * @param size scaling of robot
     * @param force maximal used force to realize motorcommand
     * @param speed factor for changing speed of robot
     * @param sphereWheels switches between spheres and  'normal' wheels
     */
    Nimm4(const OdeHandle& odeHandle, const VsgHandle& vsgHandle, const std::string& name,
          double size=1, double force=3, double speed=15, bool sphereWheels=true);

    virtual ~Nimm4(){
      destroy();
    };

    /**
     * updates the VSG nodes of the vehicle
     */
    virtual void update();


    /** sets the pose of the vehicle
        @param pose desired pose matrix
    */
    virtual void placeIntern(const vsg::dmat4& pose);

    /** returns actual sensorvalues
        @param sensors sensors scaled to [-1,1]
        @param sensornumber length of the sensor array
        @return number of actually written sensors
    */
    virtual int getSensorsIntern(double* sensors, int sensornumber);

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


  protected:
    /** creates vehicle at desired pose
        @param pose 4x4 pose matrix
    */
    virtual void create(const vsg::dmat4& pose);

    /** destroys vehicle and space
     */
    virtual void destroy();

    /** additional things for collision handling can be done here
     */
    static void mycallback(void *data, dGeomID o1, dGeomID o2);

    double length;     // chassis length
    double width;      // chassis width
    double height;     // chassis height
    double radius;     // wheel radius
    double wheelthickness; // thickness of the wheels
    bool sphereWheels; // draw spherical wheels?
    double cmass;      // chassis mass
    double wmass;      // wheel mass
    int sensorno;      // number of sensors
    int motorno;       // number of motors
    int segmentsno;    // number of motorsvehicle segments
    double speed;      // factor for adjusting speed of robot

    double max_force;  // maximal force for motors

    bool created;      // true if robot was created

    Substance wheelsubstance; // material of wheel

  };

}

#endif
