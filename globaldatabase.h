#ifndef __GLOBALDATABASE_H_
#define __GLOBALDATABASE_H_

#include "agent.h"
#include "configurablelist.h"

#ifndef NOCONFIGURATOR
namespace lpzrobots {
  class ConfiguratorProxy;
}
#endif

typedef std::vector<Agent*> AgentList;

class GlobalDataBase {
  public:
    GlobalDataBase();

    virtual ~GlobalDataBase();

    virtual AgentList& getAgents() = 0;

    template <typename Derived> struct dynamic_agent_caster {
        Agent* operator()(Derived instance) { return dynamic_cast<Agent*>(instance); }
    };

    /**
     * Creates the Configurator and, if already exists, destroys the old one.
     */
    void createConfigurator();

    /**
     * Destroys the Configurator if it was created.
     */
    void removeConfigurator();
  
    /**
     * @return true if the Configurator is open and alive
     */
    bool isConfiguratorOpen();

    ConfigurableList configs;
    
#ifndef NOCONFIGURATOR
    lpzrobots::ConfiguratorProxy * configurator;
#else
    void* configurator;
#endif
  
};

#endif /* __GLOBALDATABASE_H_ */
