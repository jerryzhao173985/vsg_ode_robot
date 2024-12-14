// ConfiguratorProxy.cpp
#include "configuratorProxy.h"
#include <iostream>

namespace lpzrobots {

static void* createConfiguratorThread(void* thread);

ConfiguratorProxy::ConfiguratorProxy(ConfigurableList& configList) 
    : configList(configList), configurator(nullptr) {
    pthread_create(&configuratorThread, NULL, createConfiguratorThread, this);
}

ConfiguratorProxy::~ConfiguratorProxy() {
    if (configurator != nullptr) {
        configurator->close();
        delete configurator;
        configurator = nullptr;
    }
}

void ConfiguratorProxy::doOnCallBack(BackCaller* source, 
    BackCaller::CallbackableType type) {
    if (type == ConfigurableList::CALLBACK_CONFIGURABLE_LIST_BEING_DELETED) {
        // unregister
        source->removeCallbackable(this, 
            ConfigurableList::CALLBACK_CONFIGURABLE_LIST_BEING_DELETED);
        source->removeCallbackable(this, 
            ConfigurableList::CALLBACK_CONFIGURABLE_LIST_MODIFIED);
        
        // delete configurator
        if (configurator != nullptr) {
            configurator->close();
            delete configurator;
            configurator = nullptr;
        }
    }
}

void ConfiguratorProxy::createConfigurator() {
    configurator = new SimpleConfigurator(configList);
    configList.addCallbackable(this, 
        ConfigurableList::CALLBACK_CONFIGURABLE_LIST_BEING_DELETED);
    configurator->show();
}

static void* createConfiguratorThread(void* thread) {
    ConfiguratorProxy* proxy = dynamic_cast<ConfiguratorProxy*>((ConfiguratorProxy*)thread);
    if (proxy) {
        proxy->createConfigurator();
    } else {
        std::cerr << "createConfiguratorThread()::Failed to create configurator" << std::endl;
    }
    return NULL;
}

} // namespace lpzrobots