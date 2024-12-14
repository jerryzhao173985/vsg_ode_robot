#include "globaldatabase.h"

#ifndef NOCONFIGURATOR
#include "configuratorProxy.h"

using namespace lpzrobots;
#endif

GlobalDataBase::GlobalDataBase() :
  configurator(0) {
}

GlobalDataBase::~GlobalDataBase() {}

void GlobalDataBase::createConfigurator() {
#ifndef NOCONFIGURATOR
  configurator = new ConfiguratorProxy(configs);
#endif
}

bool GlobalDataBase::isConfiguratorOpen(){
  return configurator!=0;
}

void GlobalDataBase::removeConfigurator() {
#ifndef NOCONFIGURATOR
  if(configurator!=0){
    delete configurator;
  }
  configurator = 0;
#endif
}
