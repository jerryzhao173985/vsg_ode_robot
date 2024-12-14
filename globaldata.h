#ifndef __GLOBALDATA_H
#define __GLOBALDATA_H

#include <vector>
#include <map>
#include "odehandle.h"
#include "odeconfig.h"
#include "vsghandle.h"
#include "sound.h"
#include "tmpobject.h"
#include "plotoption.h"
#include "globaldatabase.h"
#include "backcallervector.h"
#include "abstractobstacle.h"

class Configurable;

namespace lpzrobots {

  class OdeAgent;
  class AbstractObstacle;
  class Primitive;

  typedef std::vector<AbstractObstacle*> ObstacleList;
  typedef Configurable::configurableList ConfigList;
  typedef BackCallerVector<OdeAgent*> OdeAgentList;
  typedef std::list<Sound> SoundList;
  typedef std::list<PlotOption> PlotOptionList;
  typedef std::multimap<double, TmpObject* > TmpObjectMap;
  typedef std::list< std::pair<double, TmpObject*> > TmpObjectList;

  /**
   Data structure holding all essential global information.
   */
  class GlobalData : public GlobalDataBase {
    public:
      GlobalData() {
        sim_step = 0;
        environment = 0;
      }

      virtual ~GlobalData() {}

      VsgHandle vsgHandle;
      OdeConfig odeConfig;
      ObstacleList obstacles;
      OdeAgentList agents;
      Primitive* environment; /// < this is used to be able to attach objects to the static environment

      // Todo: the sound visualization could be done with the new TmpObjects
      SoundList sounds; ///< sound space

      PlotOptionList plotoptions; ///< plotoptions used for new agents
      std::list<Configurable*> globalconfigurables; ///< global configurables plotted by all agents

      double time;
      long int sim_step; ///< time steps since start

      /// returns the list of all agents
      virtual AgentList& getAgents();


      /// adds a temporary display item with given life duration in sec
      virtual void addTmpObject(TmpObject* i, double duration);

      /// called by Simulation to initialize tmp objects
      virtual void initializeTmpObjects(const OdeHandle& odeHandle,
                                        const VsgHandle& vsgHandle);
      /// called by Simulation to update tmp objects
      virtual void updateTmpObjects(const VsgHandle& vsgHandle);

      /** called by Simulation to removes all expired sounds and temporary objects.
          Optionally a time can be specified otherwise the internal time is used.
      */
      virtual void removeExpiredObjects(double time = -1);

      /** removes a particular temporary display item even if it is not yet expired
          @return true if it was deleted (found) */
      virtual bool removeTmpObject(TmpObject* i);


    private:

      TmpObjectList uninitializedTmpObjects;
      TmpObjectMap  tmpObjects;
      AgentList     baseAgents;
      
  };

}

#endif