#ifndef           MOTOR_H_
#define           MOTOR_H_

#include "globaldata.h"
#include "sensormotorinfoable.h"

namespace lpzrobots {

  /**
      Abstact base class for attachable motors
  */
  class Motor : public virtual SensorMotorInfoAble {
  public:
    Motor() {
    }
    virtual ~Motor() {};

    /** initialises motor with body of robot
    */
    virtual void init(Primitive* own, Joint* joint = 0)=0;

    /// return the dimensionality of this motor
    virtual int getMotorNumber() const =0;

    /** returns a list of motor names  (@see SensorMotorNaming how to change the names) */
    virtual std::list<SensorMotorInfo> getMotorInfos() const {
      return getInfos(getMotorNumber());
    };

    /** performs the actions, This is usually called in
        doInternalStuff() from the robot */
    virtual bool act(GlobalData& globaldata) = 0;

    /** sends the action commands to the motor.
        It returns the number of used values. (should be equal to
        getMotorNumber)
     */
    virtual int set(const motor* values, int length) = 0;
  };
}

#endif /* !MOTOR_H_ */
