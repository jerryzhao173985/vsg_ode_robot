#ifndef __SENSOR_H
#define __SENSOR_H

#include <list>
#include "types.h"
#include "stl_adds.h"
#include "matrix.h"
#include "globaldata.h"
#include "sensormotorinfoable.h"
#include "pos.h"

namespace lpzrobots {

  // forward declaration
  class Primitive;
  class Joint;

  /** Abstract class for sensors that can be plugged into a robot
  */
  class Sensor : public virtual SensorMotorInfoAble {
  public:
    /// defines which dimensions should be sensed. The meaning is sensor specific.
    enum Dimensions { X = 1, Y = 2, Z = 4, XY = X | Y, XZ = X | Z, YZ = Y | Z, XYZ = X | Y | Z };

    Sensor() {}
    virtual ~Sensor() {}

    /** initialises sensor with a body of robot and optionally with a joint.
        This is usually done by the robot itself (or using the Attachment())
    */
    virtual void init(Primitive* own, Joint* joint = 0) = 0;

    /** performs sense action
     */
    virtual bool sense(const GlobalData& globaldata) = 0;

    /** returns the number of sensors values produced by this sensor
     */
    virtual int getSensorNumber() const  = 0;

    /** returns a list of sensor values (usually in the range [-1,1] )
        This function should be overloaded.
        If performance matters, implement get(double*, int) and use getListOfArray to implement this.
     */
    virtual std::list<sensor> getList() const  = 0;


    /** returns a list of sensor infos  (@see SensorMotorInfoAble how to change the names etc) */
    virtual std::list<SensorMotorInfo> getSensorInfos() const {
      return getInfos(getSensorNumber());
    };


    /** to update any visual appearance
     */
    virtual void update() {};

    /** writes the sensor values (usually in the range [-1,1] )
        into the given sensor array and returns the number of sensors written.
        A default implementation based on get() is provided. Only if performance
        matters overwrite this function.
        @param sensors call by refernce array which received the values
        @param length capacity of sensors array
        @return number of sensor values written
     */
    virtual int get(sensor* sensors, int length) const {
      const std::list<sensor>& l = getList();
      assert(length>=(int)l.size());
      int n=0;
      FOREACHC(std::list<sensor>,l,s)
        sensors[n++] = *s;
      return l.size();
    };

    /// helper function for performance implementation of list<> get() based on array-get
    std::list<sensor> getListOfArray() const {
      int num = getSensorNumber();
      sensor* s = new sensor[num];
      get(s,num);
      std::list<sensor> result(s,s+num);
      delete[] s;
      return result;
    }

    /// selects the rows specified by dimensions (X->0, Y->1, Z->2)
    static std::list<sensor> selectrows(const matrix::Matrix& m, short dimensions) {
      std::list<sensor> l;
      for(int i=0; i<3; i++){
        if(( 1 <<i ) & dimensions) l += m.row(i).convertToList();
      }
      return l;
    }
    /// selects the rows specified by dimensions (X->0, Y->1, Z->2)
    static int selectrows(sensor* sensors, int length, const matrix::Matrix& m, short dimensions) {
      int len=0;
      for(int i=0; i<3; i++){
        if(( 1 << i) & dimensions)
          len+=m.row(i).convertToBuffer(sensors+len, length-len);
      }
      return len;
    }

    // parse a string with "xyz" or "XYZ" for specifing the sensors dimension
    static Dimensions parseSensorDimension(char* str){
      int val=0;
      for(unsigned int i=0; i<strlen(str); i++){
        switch(str[i]){
        case 'X':
        case 'x': val|=X; break;
        case 'Y':
        case 'y': val|=Y; break;
        case 'Z':
        case 'z': val|=Z; break;
        }
      }
      if(val==0) {
        fprintf(stderr,"parseSensorDimension:Sensor must have at least one dimension");
        val = X;
      }
      return (Dimensions)val;
    }

    // prints sensor dimensions "XYZ"
    static std::string dimensions2String(short dimensions){
      std::string s;
      if((dimensions & X) != 0) s+="X";
      if((dimensions & Y) != 0) s+="Y";
      if((dimensions & Z) != 0) s+="Z";
      return s;
    }


  };

}

#endif
