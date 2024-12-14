#ifndef __TMPOBJECT_H
#define __TMPOBJECT_H

#include "vsghandle.h"
#include "odehandle.h"
#include "pose.h"

namespace lpzrobots {

  /**
     this is the base-class for objects that exist temporarily like
     some indicator of manipulation or a message text
   */
  class TmpObject {
  public:
    TmpObject()
      : time(0) {} ;

    virtual ~TmpObject() {};
    virtual void init(const OdeHandle& odeHandle, const VsgHandle& vsgHandle) = 0;
    /// deletes the object
    virtual void deleteObject() = 0;
    /// update graphics here
    virtual void update() =0 ;

    void setExpireTime(double time) { this->time= time; }
    bool expired(double time) { return this->time < time;}

  protected:
    double time;
  };

}

#endif
