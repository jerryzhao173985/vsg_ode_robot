// ConfiguratorProxy.h
#ifndef __CONFIGURATORPROXY_H_
#define __CONFIGURATORPROXY_H_

#include "configurablelist.h"
#include "callbackable.h"
#include "simpleConfigurator.h"
#include <pthread.h>

namespace lpzrobots {

/**
 * Proxy which controls the creation process of the SimpleConfigurator
 */
class ConfiguratorProxy : public Callbackable {
public:
    ConfiguratorProxy(ConfigurableList& configList);
    virtual ~ConfiguratorProxy();
    
    virtual void doOnCallBack(BackCaller* source, 
        BackCaller::CallbackableType type = BackCaller::DEFAULT_CALLBACKABLE_TYPE);
    
    void createConfigurator();

private:
    ConfigurableList& configList;
    pthread_t configuratorThread;
    SimpleConfigurator* configurator;
};

} // namespace lpzrobots

#endif /* __CONFIGURATORPROXY_H_ */