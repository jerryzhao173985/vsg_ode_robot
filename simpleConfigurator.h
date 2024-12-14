// SimpleConfigurator.h
#ifndef __SIMPLE_CONFIGURATOR_H
#define __SIMPLE_CONFIGURATOR_H

#include "configurablelist.h"
#include <string>
#include <map>
#include <iostream>

namespace lpzrobots {

class SimpleConfigurator {
public:
    SimpleConfigurator(ConfigurableList& configList);
    virtual ~SimpleConfigurator();

    // Main interface methods
    void show();
    void close();
    void refresh();

private:
    void displayMenu();
    void listConfigurables();
    void modifyParameter();
    void saveToFile();
    void loadFromFile();
    void displayHelp();
    
    void displayConfigurable(Configurable* conf, int indent = 0);
    std::string getIndentation(int level) const;
    
    ConfigurableList& configList;
    bool running;
    
    // Helper methods
    void processUserInput();
    void displayParameterDetails(Configurable* conf);
    bool setConfigurableParameter(Configurable* conf, const std::string& paramName, const std::string& value);
};

} // namespace lpzrobots

#endif // __SIMPLE_CONFIGURATOR_H