#ifndef           SOUND_H_
#define           SOUND_H_

#include "pos.h"

namespace lpzrobots {

  class GlobalData;

  /// Object that represents a sound signal in the simulator
  class Sound {
  public:
    Sound(double time, const Pos& pos, float intensity, float frequency, void* sender);

    ~Sound();

    void createVisual(GlobalData& globalData, double visualSize, Pos visualOffset) const;

    /// nice predicate function for finding old sound signals
    struct older_than {
      older_than(double time) : time(time) {}
      double time;
      bool operator()(const Sound& s) const { return s.time < time; }
    };

    double time;
    Pos pos;    ///< emission position
    float intensity; ///< intensity 0..1
    float frequency; ///< frequency -1..1
    void* sender;    ///< pointer to the sender (can be used for self
                     ///detection)

  };

} // end namespace

#endif             /* !SOUND_H_ */
