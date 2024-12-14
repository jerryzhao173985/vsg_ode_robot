#include "sound.h"
#include "vsghandle.h"
#include "primitive.h"
#include "globaldata.h"
#include "tmpprimitive.h"
#include "globaldata.h"

namespace lpzrobots {

  Sound::Sound(double time, const Pos& pos, float intensity, float frequency, void* sender)
    : time(time), pos(pos),
      frequency(frequency), sender(sender) {
    this->intensity=std::max(std::min(intensity, 1.0f), 0.0f);
  }

  Sound::~Sound(){
  }

  void Sound::createVisual(GlobalData& globalData, double visualSize, Pos visualOffset) const {
    globalData.addTmpObject(new TmpDisplayItem(new Sphere((intensity)*visualSize),
                                               vsg::translate(pos + visualOffset),
                                               Color(255-int((frequency+1.0)*128.0),
                                                     0,int((frequency+1.0)*128.0),0.4)),
                            globalData.odeConfig.simStepSize*globalData.odeConfig.controlInterval);

  }

}
