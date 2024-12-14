#ifndef __ODECONFIG_H
#define __ODECONFIG_H

#include "configurable.h"
#include "odehandle.h"

namespace lpzrobots {

  /**
     The class $name holds the configurable parameters of the simulation environment.
  */
  class OdeConfig : public Configurable {
  public:

    // creates new instance of OdeConfig with default values
    OdeConfig();

    virtual ~OdeConfig() {}

    virtual long int getRandomSeed() const { return randomSeed; }

    virtual void setRandomSeed(long int randomSeed){
      this->randomSeed=randomSeed;
      randomSeedCopy = randomSeed;
    }

    virtual void setOdeHandle(const OdeHandle& odeHandle);

    virtual void setVideoRecordingMode(bool mode);

    virtual void calcAndSetDrawInterval(double Hz, double rtf);

    /******** CONFIGURABLE ***********/
    virtual void notifyOnChange(const paramkey& key);


  private:
    /// calculates the draw interval with simStepSize and realTimeFactor so that we have the Hz frames/sec
    int calcDrawInterval(double Hz, double rtf);

  public:
    bool videoRecordingMode;
    bool logWhileRecording;
    double simStepSize;
    int drawInterval;
    int controlInterval;
    double noise;
    double gravity;
    double cameraSpeed;
    OdeHandle odeHandle;

    double realTimeFactor;
    double fps;
  protected:
    long randomSeed;
    double randomSeedCopy;
  };

}

#endif
